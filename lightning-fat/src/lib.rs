use core::fmt::Display;

use embedded_io_async::{Error, ErrorType, Read, ReadExactError, Seek, SeekFrom, Write};

pub const SECTOR_SIZE: u32 = 512;
pub const DIR_ENTRY_SIZE: u32 = 32;

#[derive(Debug)]
pub enum FsError<E> {
    Io(E),
    InvalidFormat,
    EntryIsDirectory,
    UnexpectedEof,
    FileAlreadyExists,
    FileDoesNotExist,
    OutOfSpace,
}
impl<E: Error> From<E> for FsError<E> {
    fn from(e: E) -> Self {
        FsError::Io(e)
    }
}
impl<E: Error> From<ReadExactError<E>> for FsError<E> {
    fn from(e: ReadExactError<E>) -> Self {
        match e {
            ReadExactError::UnexpectedEof => FsError::UnexpectedEof,
            ReadExactError::Other(e) => FsError::Io(e),
        }
    }
}
impl<E: Error> Display for FsError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            FsError::Io(e) => write!(f, "I/O error: {}", e),
            FsError::InvalidFormat => write!(f, "Invalid format"),
            FsError::EntryIsDirectory => write!(f, "Entry is a directory"),
            FsError::UnexpectedEof => write!(f, "Unexpected end of file"),
            FsError::FileAlreadyExists => write!(f, "File already exists"),
            FsError::FileDoesNotExist => write!(f, "File does not exist"),
            FsError::OutOfSpace => write!(f, "Out of space"),
        }
    }
}
impl<E: Error> core::error::Error for FsError<E> {}
impl<E: Error> Error for FsError<E> {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        match self {
            FsError::Io(e) => e.kind(),
            _ => embedded_io_async::ErrorKind::Other,
        }
    }
}

pub trait ReadWriteSeek: Read + Write + Seek {}
impl<T: Read + Write + Seek> ReadWriteSeek for T {}

pub struct FileSystem<S: ReadWriteSeek> {
    storage: S,
    boot_sector: BootSector,
    fat: FileAllocationTable,
    first_cluster_offset: u64,
    cluster_size: u32,
    total_clusters: u32,
}

impl<S: ReadWriteSeek> FileSystem<S> {
    pub async fn new(mut storage: S) -> Result<Self, FsError<S::Error>> {
        // TODO: check signatures: MBR, BS, FAT32 OEM, etc.

        const MBR_OFFSET_PARTITION_1_ENTRY: u64 = 446;
        let partition_entry = {
            let mut data = [0u8; 16];
            storage
                .seek(SeekFrom::Start(MBR_OFFSET_PARTITION_1_ENTRY))
                .await?;
            storage.read_exact(&mut data).await?;
            PartitionEntry {
                typ: data[4],
                starting_lba: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
                // size_in_sectors: u32::from_le_bytes([data[12], data[13], data[14], data[15]]),
            }
        };
        if partition_entry.typ != 0x0C {
            return Err(FsError::InvalidFormat);
        }

        let partition_offset = partition_entry.starting_lba as u64 * SECTOR_SIZE as u64;

        let boot_sector = {
            let mut data = [0u8; 50];
            storage.seek(SeekFrom::Start(partition_offset)).await?;
            storage.read_exact(&mut data).await?;
            BootSector {
                bytes_per_sector: u16::from_le_bytes([data[11], data[12]]),
                sectors_per_cluster: data[13],
                reserved_sectors: u16::from_le_bytes([data[14], data[15]]),
                number_of_fats: data[16],
                total_sectors: u32::from_le_bytes([data[32], data[33], data[34], data[35]]),
                sectors_per_fat: u32::from_le_bytes([data[36], data[37], data[38], data[39]]),
                root_directory_cluster: u32::from_le_bytes([
                    data[44], data[45], data[46], data[47],
                ]),
                fs_info_sector: u16::from_le_bytes([data[48], data[49]]),
            }
        };
        if boot_sector.bytes_per_sector as u32 != SECTOR_SIZE {
            return Err(FsError::InvalidFormat);
        }

        let fs_info_offset =
            partition_offset + boot_sector.fs_info_sector as u64 * SECTOR_SIZE as u64;

        // Reset fs info so that we don't have to worry about it being out of date.
        const FS_INFO_DATA_OFFSET: u64 = 488;
        storage
            .seek(SeekFrom::Start(fs_info_offset + FS_INFO_DATA_OFFSET))
            .await?;
        storage.write_all(&[0xFF; 8]).await?;

        let fat = FileAllocationTable {
            offset: partition_offset + boot_sector.reserved_sectors as u64 * SECTOR_SIZE as u64,
            entries_count: boot_sector.sectors_per_fat * (boot_sector.bytes_per_sector as u32 / 4),
            copies: boot_sector.number_of_fats,
        };

        let cluster_size = boot_sector.sectors_per_cluster as u32 * SECTOR_SIZE;
        let first_cluster_offset = fat.offset
            + boot_sector.number_of_fats as u64
                * boot_sector.sectors_per_fat as u64
                * SECTOR_SIZE as u64;

        let total_clusters = (boot_sector.total_sectors
            - boot_sector.reserved_sectors as u32
            - fat.copies as u32 * boot_sector.sectors_per_fat)
            / boot_sector.sectors_per_cluster as u32;

        Ok(Self {
            storage,
            boot_sector,
            fat,
            cluster_size,
            first_cluster_offset,
            total_clusters,
        })
    }

    pub fn root_dir(&self) -> DirHandle {
        DirHandle {
            start_cluster: self.boot_sector.root_directory_cluster,
        }
    }

    pub async fn file_exists(
        &mut self,
        dir: &DirHandle,
        name: &ShortFileName,
    ) -> Result<bool, FsError<S::Error>> {
        let mut found = false;
        let mut it = self.iter_dir(dir);
        while let Some(dir_entry) = it.next().await? {
            if dir_entry.name == *name {
                found = true;
                break;
            }
        }
        Ok(found)
    }

    pub async fn create_file(
        &mut self,
        dir: &DirHandle,
        name: &ShortFileName,
    ) -> Result<DirEntry, FsError<S::Error>> {
        if self.file_exists(dir, name).await? {
            return Err(FsError::FileAlreadyExists);
        }

        let mut unused_entry_location = None;

        let mut it = DirEntryIterInternal::new(self, dir);
        while let Some(state) = it.next().await? {
            match state {
                DirEntryState::Present(_) => (),
                DirEntryState::Unused(location) => {
                    unused_entry_location = Some(location);
                },
            }
        }

        let (cluster, offset) = if let Some(location) = unused_entry_location {
            location
        } else {
            let last_cluster = it.current_cluster;
            let next_cluster = self.alloc_chained_cluster(last_cluster).await?;
            self.zero_cluster(next_cluster).await?;
            (next_cluster, 0)
        };
        let dir_entry_offset = self.cluster_offset(cluster)? + offset as u64;

        let dir_entry = DirEntry {
            name: name.clone(),
            attr: DirEntryAttr::ARCHIVE,
            file_size: 0,
            start_cluster: 0,
        };

        let mut data = [0u8; DIR_ENTRY_SIZE as usize];
        data[0..11].copy_from_slice(&dir_entry.name.raw);
        data[11] = dir_entry.attr.bits();
        // the rest of the fields are zero on creation

        self.storage.seek(SeekFrom::Start(dir_entry_offset)).await?;
        self.storage.write_all(&data).await?;
        self.storage.flush().await?;

        Ok(dir_entry)
    }

    pub async fn open_file(
        &mut self,
        dir: &DirHandle,
        name: &ShortFileName,
    ) -> Result<DirEntry, FsError<S::Error>> {
        let mut it = self.iter_dir(dir);
        while let Some(dir_entry) = it.next().await? {
            if dir_entry.name == *name {
                return Ok(dir_entry);
            }
        }
        Err(FsError::FileDoesNotExist)
    }

    pub fn iter_dir<'a>(&'a mut self, dir: &DirHandle) -> DirEntryIter<'a, S> {
        DirEntryIter {
            inner: DirEntryIterInternal::new(self, dir),
        }
    }

    pub fn file_reader<'a>(
        &'a mut self,
        entry: &DirEntry,
    ) -> Result<FileReader<'a, S>, FsError<S::Error>> {
        if entry.is_directory() {
            return Err(FsError::EntryIsDirectory);
        }
        Ok(FileReader {
            fs: self,
            current_cluster: entry.start_cluster,
            offset_in_cluster: 0,
            bytes_remaining: entry.file_size,
        })
    }

    fn cluster_offset(&self, cluster: u32) -> Result<u64, FsError<S::Error>> {
        if cluster < 2 || cluster >= self.total_clusters + 2 {
            return Err(FsError::InvalidFormat);
        }
        Ok(self.first_cluster_offset + (cluster as u64 - 2) * self.cluster_size as u64)
    }

    async fn lookup_next_cluster(
        &mut self,
        cluster: u32,
    ) -> Result<Option<u32>, FsError<S::Error>> {
        let fat_entry_offset = self.fat.offset + cluster as u64 * 4;
        self.storage.seek(SeekFrom::Start(fat_entry_offset)).await?;
        let mut buf = [0u8; 4];
        self.storage.read_exact(&mut buf).await?;
        let fat_entry = FatEntry::new(u32::from_le_bytes(buf));
        match fat_entry {
            FatEntry::Next(next_cluster) => Ok(Some(next_cluster)),
            _ => Ok(None),
        }
    }

    async fn alloc_chained_cluster(&mut self, prev_cluster: u32) -> Result<u32, FsError<S::Error>> {
        // TODO: use FS INFO Nxt_Free to not search from the beginning
        // TODO: prev cluster not checked for being greater than two

        let mut free_cluster = None;

        self.storage
            .seek(SeekFrom::Start(self.fat.offset + 2 * 4))
            .await?; // skip first two entries (reserved)

        for cluster in 2..self.fat.entries_count {
            let mut buf = [0u8; 4];
            self.storage.read_exact(&mut buf).await?;

            let entry = FatEntry::new(u32::from_le_bytes(buf));
            if let FatEntry::Free = entry {
                free_cluster = Some(cluster);
                break;
            }
        }

        if let Some(next_cluster) = free_cluster {
            for fat_id in 0..self.fat.copies {
                let offset = self.fat.offset
                    + fat_id as u64 * self.boot_sector.sectors_per_fat as u64 * SECTOR_SIZE as u64;
                self.storage
                    .seek(SeekFrom::Start(offset + 4 * next_cluster as u64))
                    .await?;
                self.storage
                    .write_all(&FatEntry::EndOfChain.raw().to_le_bytes())
                    .await?;
                self.storage
                    .seek(SeekFrom::Start(offset + 4 * prev_cluster as u64))
                    .await?;
                self.storage
                    .write_all(&FatEntry::Next(next_cluster).raw().to_le_bytes())
                    .await?;
            }
            self.storage.flush().await?;
            Ok(next_cluster)
        } else {
            Err(FsError::OutOfSpace)
        }
    }

    async fn zero_cluster(&mut self, cluster: u32) -> Result<(), FsError<S::Error>> {
        let mut offset = self.cluster_offset(cluster)?;
        let end_offset = offset + self.cluster_size as u64;

        // NOTE: assumes self.cluster_size % buf.len() == 0, which is true if the format is ok

        self.storage.seek(SeekFrom::Start(offset)).await?;

        let buf = [0u8; 64];
        while offset < end_offset {
            self.storage.write_all(&buf).await?;
            offset += buf.len() as u64;
        }
        self.storage.flush().await?;
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct DirHandle {
    start_cluster: u32,
}

// TODO: store global offset
#[derive(Clone, Debug)]
pub struct DirEntry {
    name: ShortFileName,
    attr: DirEntryAttr,
    file_size: u32,
    start_cluster: u32,
}

impl DirEntry {
    pub fn name(&self) -> &ShortFileName {
        &self.name
    }

    pub fn attr(&self) -> DirEntryAttr {
        self.attr
    }

    pub fn is_directory(&self) -> bool {
        self.attr.contains(DirEntryAttr::DIRECTORY)
    }

    pub fn size(&self) -> u32 {
        self.file_size
    }
}

bitflags::bitflags! {
    #[derive(Clone, Copy, Debug)]
    pub struct DirEntryAttr: u8 {
        const READ_ONLY = 0x01;
        const HIDDEN = 0x02;
        const SYSTEM = 0x04;
        const VOLUME_ID = 0x08;
        const DIRECTORY = 0x10;
        const ARCHIVE = 0x20;
        const LONG_NAME = Self::READ_ONLY.bits() | Self::HIDDEN.bits() | Self::SYSTEM.bits() | Self::VOLUME_ID.bits();
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ShortFileName {
    raw: [u8; 11],
}

impl ShortFileName {
    pub fn new(name: &[u8]) -> Option<Self> {
        // TODO: name validation

        let dot_pos = name.iter().position(|&x| x == b'.');

        let (name_part, ext_part) = match dot_pos {
            Some(pos) => (&name[..pos], &name[pos + 1..]),
            None => (name, &b""[..]),
        };

        if name_part.is_empty() || name_part.len() > 8 || ext_part.len() > 3 {
            return None;
        }

        let mut raw = [b' '; 11];
        raw[..name_part.len()].copy_from_slice(name_part);
        raw[8..][..ext_part.len()].copy_from_slice(ext_part);
        Some(Self { raw })
    }

    pub fn base_name(&self) -> &[u8] {
        let name_len = self.raw[0..8]
            .iter()
            .rposition(|x| *x != b' ')
            .map_or(0, |p| p + 1);
        &self.raw[..name_len]
    }

    pub fn ext(&self) -> &[u8] {
        let ext_len = self.raw[8..11]
            .iter()
            .rposition(|x| *x != b' ')
            .map_or(0, |p| p + 1);
        &self.raw[8..][..ext_len]
    }
}

impl Display for ShortFileName {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let base_name = self.base_name();
        let ext = self.ext();
        if !ext.is_empty() {
            write!(
                f,
                "{}.{}",
                core::str::from_utf8(base_name).unwrap_or(""),
                core::str::from_utf8(ext).unwrap_or("")
            )
        } else {
            write!(f, "{}", core::str::from_utf8(base_name).unwrap_or(""))
        }
    }
}

pub struct DirEntryIter<'a, S: ReadWriteSeek> {
    inner: DirEntryIterInternal<'a, S>,
}

impl<'a, S: ReadWriteSeek> DirEntryIter<'a, S> {
    pub async fn next(&mut self) -> Result<Option<DirEntry>, FsError<S::Error>> {
        while let Some(state) = self.inner.next().await? {
            match state {
                DirEntryState::Present(dir_entry) => {
                    if !dir_entry.attr.contains(DirEntryAttr::LONG_NAME)
                        && !dir_entry.attr.contains(DirEntryAttr::VOLUME_ID)
                    {
                        return Ok(Some(dir_entry));
                    }
                }
                DirEntryState::Unused(_) => (),
            }
        }
        Ok(None)
    }
}

#[derive(Clone, Debug)]
enum DirEntryState {
    Present(DirEntry),
    Unused((u32, u32)),
}

struct DirEntryIterInternal<'a, S: ReadWriteSeek> {
    fs: &'a mut FileSystem<S>,
    current_cluster: u32,
    offset_in_cluster: u32,
}

impl<'a, S: ReadWriteSeek> DirEntryIterInternal<'a, S> {
    fn new(fs: &'a mut FileSystem<S>, dir: &DirHandle) -> Self {
        Self {
            fs,
            current_cluster: dir.start_cluster,
            offset_in_cluster: 0,
        }
    }

    async fn next(&mut self) -> Result<Option<DirEntryState>, FsError<S::Error>> {
        let dir_entry_offset =
            self.fs.cluster_offset(self.current_cluster)? + self.offset_in_cluster as u64;

        self.fs
            .storage
            .seek(SeekFrom::Start(dir_entry_offset))
            .await?;

        let mut data = [0u8; 32];
        self.fs.storage.read_exact(&mut data).await?;

        let state = match data[0] {
            0x00 | 0xE5 => DirEntryState::Unused((self.current_cluster, self.offset_in_cluster)),
            _ => DirEntryState::Present(DirEntry {
                name: ShortFileName {
                    raw: data[0..11].try_into().unwrap(),
                },
                attr: DirEntryAttr::from_bits_truncate(data[11]),
                file_size: u32::from_le_bytes([data[28], data[29], data[30], data[31]]),
                start_cluster: ((u16::from_le_bytes([data[20], data[21]]) as u32) << 16)
                    | u16::from_le_bytes([data[26], data[27]]) as u32,
            }),
        };

        // if not the last entry, we advance
        self.offset_in_cluster += DIR_ENTRY_SIZE;

        if self.offset_in_cluster >= self.fs.cluster_size {
            if let Some(next_cluster) = self.fs.lookup_next_cluster(self.current_cluster).await? {
                self.current_cluster = next_cluster;
                self.offset_in_cluster = 0;
            } else {
                return Ok(None);
            }
        }

        Ok(Some(state))
    }
}

pub struct FileReader<'a, S: ReadWriteSeek> {
    fs: &'a mut FileSystem<S>,
    current_cluster: u32,
    offset_in_cluster: u32,
    bytes_remaining: u32,
}

impl<'a, S: ReadWriteSeek> ErrorType for FileReader<'a, S> {
    type Error = FsError<S::Error>;
}

impl<'a, S: ReadWriteSeek> Read for FileReader<'a, S> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, FsError<S::Error>> {
        if self.bytes_remaining == 0 {
            return Ok(0);
        }

        let read_amount = buf.len().min(self.bytes_remaining as usize);
        let mut read_until_now = 0;

        while read_until_now < read_amount {
            if self.offset_in_cluster >= self.fs.cluster_size {
                match self.fs.lookup_next_cluster(self.current_cluster).await? {
                    Some(next) => {
                        self.current_cluster = next;
                        self.offset_in_cluster = 0;
                    }
                    None => {
                        return Err(FsError::UnexpectedEof);
                    }
                }
            }

            let available_in_cluster = self.fs.cluster_size - self.offset_in_cluster;
            let chunk = (read_amount - read_until_now).min(available_in_cluster as usize);

            let offset =
                self.fs.cluster_offset(self.current_cluster)? + self.offset_in_cluster as u64;
            self.fs.storage.seek(SeekFrom::Start(offset)).await?;
            self.fs
                .storage
                .read_exact(&mut buf[read_until_now..][..chunk])
                .await?;

            read_until_now += chunk;
            self.offset_in_cluster += chunk as u32;
            self.bytes_remaining -= chunk as u32;
        }

        Ok(read_until_now)
    }
}

#[derive(Debug)]
struct PartitionEntry {
    typ: u8,
    starting_lba: u32,
}

#[derive(Debug)]
struct BootSector {
    bytes_per_sector: u16,
    sectors_per_cluster: u8,
    reserved_sectors: u16,
    number_of_fats: u8,
    total_sectors: u32,
    sectors_per_fat: u32,
    root_directory_cluster: u32,
    fs_info_sector: u16,
}

struct FileAllocationTable {
    offset: u64,
    entries_count: u32,
    copies: u8,
}

enum FatEntry {
    Free,
    Next(u32),
    EndOfChain,
    BadCluster,
}

impl FatEntry {
    pub fn new(value: u32) -> Self {
        match value {
            0x00000000 => FatEntry::Free,
            0x0FFFFFF8..=0x0FFFFFFF => FatEntry::EndOfChain,
            0x0FFFFFF7 => FatEntry::BadCluster,
            _ => FatEntry::Next(value & 0x0FFFFFFF),
        }
    }

    pub fn raw(self) -> u32 {
        match self {
            FatEntry::Free => 0x00000000,
            FatEntry::Next(cluster) => cluster & 0x0FFFFFFF,
            FatEntry::EndOfChain => 0x0FFFFFFF,
            FatEntry::BadCluster => 0x0FFFFFF7,
        }
    }
}
