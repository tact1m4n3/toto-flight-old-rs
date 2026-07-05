#![no_std]

use core::fmt::Display;
use core::ops::RangeInclusive;

use embedded_io_async::{Error, ErrorType, Read, ReadExactError, Seek, SeekFrom, Write};

const SECTOR_SIZE: u32 = 512;
const DIR_ENTRY_SIZE: u32 = 32;

#[derive(Debug)]
pub enum FsError<E> {
    Io(E),
    InvalidFormat,
    EntryIsDirectory,
    UnexpectedEof,
    FileAlreadyExists,
    FileDoesNotExist,
    OutOfSpace,
    FileAlreadyContainsData,
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
            FsError::FileAlreadyContainsData => write!(f, "File already contains data"),
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
    cluster_size: u32,

    root_dir_cluster: u32,

    fat_offset: u64,
    fat_entries: u32,
    fat_copies: u8,
    fat_stride: u64,

    first_data_cluster_offset: u64,
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

        let cluster_size = boot_sector.sectors_per_cluster as u32 * SECTOR_SIZE;

        let fat_offset =
            partition_offset + boot_sector.reserved_sectors as u64 * SECTOR_SIZE as u64;
        let fat_entries = boot_sector.sectors_per_fat * (SECTOR_SIZE / 4);
        let fat_copies = boot_sector.number_of_fats;
        let fat_stride = boot_sector.sectors_per_fat as u64 * SECTOR_SIZE as u64;

        let first_data_cluster_offset = fat_offset
            + boot_sector.number_of_fats as u64
                * boot_sector.sectors_per_fat as u64
                * SECTOR_SIZE as u64;

        if boot_sector.root_directory_cluster < 2
            || boot_sector.root_directory_cluster >= fat_entries
        {
            return Err(FsError::InvalidFormat);
        }

        Ok(Self {
            storage,
            cluster_size,
            root_dir_cluster: boot_sector.root_directory_cluster,
            fat_offset,
            fat_entries,
            fat_copies,
            fat_stride,
            first_data_cluster_offset,
        })
    }

    pub fn root_dir(&self) -> DirHandle {
        DirHandle {
            start_cluster: self.root_dir_cluster,
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
        name: ShortFileName,
    ) -> Result<DirEntry, FsError<S::Error>> {
        if self.file_exists(dir, &name).await? {
            return Err(FsError::FileAlreadyExists);
        }

        let mut unused_entry_location = None;

        let mut it = DirEntryIterInternal::new(self, dir);
        while let Some(state) = it.next().await? {
            match state {
                DirEntryState::Present(_) => (),
                DirEntryState::Unused(location) => {
                    unused_entry_location = Some(location);
                    break;
                }
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
        let dir_entry_offset = self.cluster_offset(cluster) + offset as u64;

        let dir_entry = DirEntry {
            offset: dir_entry_offset,
            name,
            attr: DirEntryAttr::ARCHIVE,
            file_size: 0,
            start_cluster: None,
        };
        self.update_dir_entry(&dir_entry).await?;
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

    pub async fn get_or_create_cluster_allocator_file(
        &mut self,
        free_clusters: u32,
        file_size: RangeInclusive<u32>,
    ) -> Result<AllocFile, FsError<S::Error>> {
        let root_dir = self.root_dir();
        let name = ShortFileName::new(b"FREE").unwrap();

        let mut dir_entry = if let Ok(dir_entry) = self.open_file(&root_dir, &name).await {
            dir_entry
        } else {
            self.create_file(&root_dir, name).await?
        };

        // TODO: verification

        // if the file is already at max size we don't need to allocate more clusters
        if dir_entry.file_size >= *file_size.end() {
            return Ok(AllocFile { dir_entry });
        }

        let end_cluster = if let Some(start_cluster) = dir_entry.start_cluster {
            self.search_end_of_chain_cluster(start_cluster).await?
        } else {
            let mut maybe_start_cluster = None;
            let mut remaining_skip = free_clusters;
            let mut contiguous_size = 0;

            for current_cluster in 2..self.fat_entries {
                if matches!(
                    self.lookup_fat_entry(current_cluster).await?,
                    FatEntry::Free
                ) {
                    if remaining_skip == 0 {
                        if contiguous_size == 0 {
                            maybe_start_cluster = Some(current_cluster);
                        }
                        contiguous_size += self.cluster_size;
                        if contiguous_size >= *file_size.start() {
                            break;
                        }
                    } else {
                        remaining_skip -= 1;
                    }
                } else {
                    contiguous_size = 0;
                    maybe_start_cluster = None;
                }
            }

            let start_cluster = maybe_start_cluster.ok_or(FsError::OutOfSpace)?;
            dir_entry.start_cluster = Some(start_cluster);
            dir_entry.file_size = self.cluster_size;
            self.update_fat_entry(start_cluster, FatEntry::EndOfChain)
                .await?;
            start_cluster
        };

        let mut prev_cluster = end_cluster;
        for current_cluster in end_cluster + 1..self.fat_entries {
            if let Some(new_size) = dir_entry.file_size.checked_add(self.cluster_size)
                && new_size <= *file_size.end()
            {
                dir_entry.file_size = new_size;
            } else {
                break;
            }
            if !matches!(
                self.lookup_fat_entry(current_cluster).await?,
                FatEntry::Free
            ) {
                break;
            }

            self.update_fat_entry(prev_cluster, FatEntry::Next(current_cluster))
                .await?;
            self.update_fat_entry(current_cluster, FatEntry::EndOfChain)
                .await?;
            prev_cluster = current_cluster;
        }

        self.update_dir_entry(&dir_entry).await?;
        self.storage.flush().await?;

        Ok(AllocFile { dir_entry })
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

    pub fn file_writer<'a>(
        &'a mut self,
        dir_entry: &'a mut DirEntry,
        alloc_file: &'a mut AllocFile,
    ) -> Result<FileWriter<'a, S>, FsError<S::Error>> {
        FileWriter::new(self, dir_entry, alloc_file)
    }

    fn cluster_offset(&self, cluster: u32) -> u64 {
        self.first_data_cluster_offset + (cluster as u64 - 2) * self.cluster_size as u64
    }

    async fn lookup_fat_entry(&mut self, cluster: u32) -> Result<FatEntry, FsError<S::Error>> {
        let fat_entry_offset = self.fat_offset + cluster as u64 * 4;
        self.storage.seek(SeekFrom::Start(fat_entry_offset)).await?;
        let mut buf = [0u8; 4];
        self.storage.read_exact(&mut buf).await?;
        let fat_entry = FatEntry::new(u32::from_le_bytes(buf));
        // validate entry
        if let FatEntry::Next(next_cluster) = fat_entry
            && (next_cluster < 2 || next_cluster >= self.fat_entries)
        {
            return Err(FsError::InvalidFormat);
        }
        Ok(fat_entry)
    }

    async fn update_fat_entry(
        &mut self,
        cluster: u32,
        entry: FatEntry,
    ) -> Result<(), FsError<S::Error>> {
        for fat_id in 0..self.fat_copies {
            let offset = self.fat_offset + fat_id as u64 * self.fat_stride;
            self.storage
                .seek(SeekFrom::Start(offset + cluster as u64 * 4))
                .await?;
            self.storage.write_all(&entry.raw().to_le_bytes()).await?;
        }
        Ok(())
    }

    async fn search_end_of_chain_cluster(
        &mut self,
        start_cluster: u32,
    ) -> Result<u32, FsError<S::Error>> {
        let mut current_cluster = start_cluster;
        loop {
            match self.lookup_fat_entry(current_cluster).await? {
                FatEntry::Next(next_cluster) => {
                    current_cluster = next_cluster;
                }
                FatEntry::EndOfChain => {
                    return Ok(current_cluster);
                }
                _ => {
                    return Err(FsError::InvalidFormat);
                }
            }
        }
    }

    async fn update_dir_entry(&mut self, entry: &DirEntry) -> Result<(), FsError<S::Error>> {
        let mut data = [0u8; DIR_ENTRY_SIZE as usize];
        data[0..11].copy_from_slice(&entry.name.raw);
        data[11] = entry.attr.bits();
        if let Some(start_cluster) = entry.start_cluster {
            data[20..22].copy_from_slice(&((start_cluster >> 16) as u16).to_le_bytes());
            data[26..28].copy_from_slice(&(start_cluster as u16).to_le_bytes());
        }
        data[28..32].copy_from_slice(&entry.file_size.to_le_bytes());

        self.storage.seek(SeekFrom::Start(entry.offset)).await?;
        self.storage.write_all(&data).await?;

        Ok(())
    }

    async fn alloc_chained_cluster(&mut self, prev_cluster: u32) -> Result<u32, FsError<S::Error>> {
        // TODO: use FS INFO Nxt_Free to not search from the beginning

        let mut free_cluster = None;

        for cluster in 2..self.fat_entries {
            if self.lookup_fat_entry(cluster).await? == FatEntry::Free {
                free_cluster = Some(cluster);
                break;
            }
        }

        if let Some(next_cluster) = free_cluster {
            self.update_fat_entry(next_cluster, FatEntry::EndOfChain)
                .await?;
            self.update_fat_entry(prev_cluster, FatEntry::Next(next_cluster))
                .await?;
            self.storage.flush().await?;
            Ok(next_cluster)
        } else {
            Err(FsError::OutOfSpace)
        }
    }

    async fn zero_cluster(&mut self, cluster: u32) -> Result<(), FsError<S::Error>> {
        let mut offset = self.cluster_offset(cluster);
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

#[derive(Clone, Debug)]
pub struct AllocFile {
    dir_entry: DirEntry,
}

impl AllocFile {
    pub fn dir_entry(&self) -> &DirEntry {
        &self.dir_entry
    }
}

#[derive(Clone, Debug)]
pub struct DirEntry {
    offset: u64,

    name: ShortFileName,
    attr: DirEntryAttr,
    file_size: u32,
    start_cluster: Option<u32>,
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
        if self.offset_in_cluster >= self.fs.cluster_size {
            if let FatEntry::Next(next_cluster) =
                self.fs.lookup_fat_entry(self.current_cluster).await?
            {
                self.current_cluster = next_cluster;
                self.offset_in_cluster = 0;
            } else {
                return Ok(None);
            }
        }

        let dir_entry_offset =
            self.fs.cluster_offset(self.current_cluster) + self.offset_in_cluster as u64;

        self.fs
            .storage
            .seek(SeekFrom::Start(dir_entry_offset))
            .await?;

        let mut data = [0u8; 32];
        self.fs.storage.read_exact(&mut data).await?;

        let state = match data[0] {
            0x00 | 0xE5 => DirEntryState::Unused((self.current_cluster, self.offset_in_cluster)),
            _ => {
                let start_cluster = ((u16::from_le_bytes([data[20], data[21]]) as u32) << 16)
                    | u16::from_le_bytes([data[26], data[27]]) as u32;
                DirEntryState::Present(DirEntry {
                    offset: dir_entry_offset,
                    name: ShortFileName {
                        raw: data[0..11].try_into().unwrap(),
                    },
                    attr: DirEntryAttr::from_bits_truncate(data[11]),
                    file_size: u32::from_le_bytes([data[28], data[29], data[30], data[31]]),
                    start_cluster: if start_cluster != 0
                        && (start_cluster >= 2 && start_cluster <= self.fs.fat_entries)
                    {
                        Some(start_cluster)
                    } else {
                        None
                    },
                })
            }
        };

        self.offset_in_cluster += DIR_ENTRY_SIZE;

        Ok(Some(state))
    }
}

pub struct FileReader<'a, S: ReadWriteSeek> {
    fs: &'a mut FileSystem<S>,
    current_cluster: Option<u32>,
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
        let Some(current_cluster) = self.current_cluster else {
            return Ok(0);
        };

        let max_read_count = buf.len().min(self.bytes_remaining as usize);

        let available_in_cluster = self.fs.cluster_size - self.offset_in_cluster;
        let read_count = max_read_count.min(available_in_cluster as usize);

        let offset = self.fs.cluster_offset(current_cluster) + self.offset_in_cluster as u64;
        self.fs.storage.seek(SeekFrom::Start(offset)).await?;
        self.fs.storage.read_exact(&mut buf[..read_count]).await?;

        self.offset_in_cluster += read_count as u32;
        self.bytes_remaining -= read_count as u32;

        if self.offset_in_cluster >= self.fs.cluster_size {
            self.current_cluster = self.fs.lookup_fat_entry(current_cluster).await?.get_next();
            self.offset_in_cluster = 0;
        }

        Ok(read_count)
    }
}

pub struct FileWriter<'a, S: ReadWriteSeek> {
    fs: &'a mut FileSystem<S>,
    dir_entry: &'a mut DirEntry,
    alloc_file: &'a mut AllocFile,
    current_cluster: Option<u32>,
    offset_in_cluster: u32,
}

impl<'a, S: ReadWriteSeek> FileWriter<'a, S> {
    pub(crate) fn new(
        fs: &'a mut FileSystem<S>,
        dir_entry: &'a mut DirEntry,
        alloc_file: &'a mut AllocFile,
    ) -> Result<Self, FsError<S::Error>> {
        if dir_entry.is_directory() {
            return Err(FsError::EntryIsDirectory);
        }
        if dir_entry.start_cluster.is_some() {
            return Err(FsError::FileAlreadyContainsData);
        }

        Ok(Self {
            fs,
            dir_entry,
            alloc_file,
            current_cluster: None,
            offset_in_cluster: 0,
        })
    }

    pub async fn close(self) -> Result<(), FsError<S::Error>> {
        if let Some(current_cluster) = self.current_cluster {
            self.fs.update_dir_entry(self.dir_entry).await?;
            if self.alloc_file.dir_entry.file_size == 0 {
                self.alloc_file.dir_entry.start_cluster = None;
            } else {
                self.alloc_file.dir_entry.start_cluster = Some(current_cluster + 1);
            }
            self.fs.update_dir_entry(&self.alloc_file.dir_entry).await?;

            self.fs
                .update_fat_entry(current_cluster, FatEntry::EndOfChain)
                .await?;

            self.fs.storage.flush().await?;
        }

        Ok(())
    }
}

impl<'a, S: ReadWriteSeek> ErrorType for FileWriter<'a, S> {
    type Error = FsError<S::Error>;
}

impl<'a, S: ReadWriteSeek> Write for FileWriter<'a, S> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, FsError<S::Error>> {
        const MAX_FILE_SIZE: u32 = u32::MAX;

        if buf.is_empty() {
            return Ok(0);
        }

        if self.dir_entry.file_size == MAX_FILE_SIZE {
            return Err(FsError::OutOfSpace);
        }

        let current_cluster = if let Some(mut current_cluster) = self.current_cluster {
            if self.offset_in_cluster >= self.fs.cluster_size {
                self.alloc_file.dir_entry.file_size = self
                    .alloc_file
                    .dir_entry
                    .file_size
                    .checked_sub(self.fs.cluster_size)
                    .ok_or(FsError::OutOfSpace)?;
                current_cluster += 1;
                self.current_cluster = Some(current_cluster);
                self.offset_in_cluster = 0;
            }
            current_cluster
        } else {
            self.alloc_file.dir_entry.file_size = self
                .alloc_file
                .dir_entry
                .file_size
                .checked_sub(self.fs.cluster_size)
                .ok_or(FsError::OutOfSpace)?;
            let start_cluster = self
                .alloc_file
                .dir_entry
                .start_cluster
                .ok_or(FsError::InvalidFormat)?;
            self.current_cluster = Some(start_cluster);
            self.dir_entry.start_cluster = Some(start_cluster);
            self.offset_in_cluster = 0;
            start_cluster
        };

        let max_write_count = buf
            .len()
            .min((MAX_FILE_SIZE - self.dir_entry.file_size) as usize);
        let remaining_in_cluster = self.fs.cluster_size - self.offset_in_cluster;
        let write_count = max_write_count.min(remaining_in_cluster as usize);

        let offset = self.fs.cluster_offset(current_cluster) + self.offset_in_cluster as u64;
        self.fs.storage.seek(SeekFrom::Start(offset)).await?;
        self.fs.storage.write_all(&buf[..write_count]).await?;

        self.offset_in_cluster += write_count as u32;
        self.dir_entry.file_size += write_count as u32;

        Ok(write_count)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
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
    sectors_per_fat: u32,
    root_directory_cluster: u32,
    fs_info_sector: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

    pub fn get_next(self) -> Option<u32> {
        if let FatEntry::Next(next_cluster) = self {
            Some(next_cluster)
        } else {
            None
        }
    }
}
