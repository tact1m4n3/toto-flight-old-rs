use core::{fmt::Display, ops::ControlFlow};

use embedded_io_async::{Error, ErrorType, Read, ReadExactError, Seek, SeekFrom, Write};

pub const SECTOR_SIZE: u32 = 512;

#[derive(Debug)]
pub enum FsError<E> {
    Io(E),
    InvalidFormat,
    InvalidCluster,
    EntryIsDirectory,
    UnexpectedEof,
    ZeroRead,
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
            FsError::InvalidCluster => write!(f, "Invalid cluster"),
            FsError::EntryIsDirectory => write!(f, "Entry is a directory"),
            FsError::UnexpectedEof => write!(f, "Unexpected end of file"),
            FsError::ZeroRead => write!(f, "Zero bytes read"),
        }
    }
}
impl<E: Error> core::error::Error for FsError<E> {}
impl<E: Error> Error for FsError<E> {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        match self {
            FsError::Io(e) => e.kind(),
            FsError::InvalidFormat => embedded_io_async::ErrorKind::Other,
            FsError::InvalidCluster => embedded_io_async::ErrorKind::Other,
            FsError::EntryIsDirectory => embedded_io_async::ErrorKind::Other,
            FsError::UnexpectedEof => embedded_io_async::ErrorKind::Other,
            FsError::ZeroRead => embedded_io_async::ErrorKind::Other,
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
}

impl<S: ReadWriteSeek> FileSystem<S> {
    pub async fn new(mut storage: S) -> Result<Self, FsError<S::Error>> {
        const MBR_OFFSET_PARTITION_1_ENTRY: u64 = 446;
        storage
            .seek(SeekFrom::Start(MBR_OFFSET_PARTITION_1_ENTRY))
            .await?;
        let partition_entry = {
            let mut data = [0u8; 16];
            storage.read_exact(&mut data).await?;
            PartitionEntry {
                typ: data[4],
                starting_lba: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
                size_in_sectors: u32::from_le_bytes([data[12], data[13], data[14], data[15]]),
            }
        };
        if partition_entry.typ != 0x0C {
            return Err(FsError::InvalidFormat);
        }

        let partition_offset = partition_entry.starting_lba as u64 * SECTOR_SIZE as u64;
        storage.seek(SeekFrom::Start(partition_offset)).await?;

        let boot_sector = {
            let mut data = [0u8; 50];
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

        Ok(Self {
            storage,
            boot_sector,
            fat,
            cluster_size,
            first_cluster_offset,
        })
    }

    pub fn root_dir(&self) -> RawDirectory {
        RawDirectory {
            start_cluster: self.boot_sector.root_directory_cluster,
        }
    }

    pub async fn iter_dir<F: FnMut(&DirEntry) -> ControlFlow<()>>(
        &mut self,
        dir: &RawDirectory,
        mut f: F,
    ) -> Result<(), FsError<S::Error>> {
        let mut current_cluster = dir.start_cluster;
        let mut current_cluster_offset = self.cluster_offset(current_cluster)?;
        let mut offset_in_cluster = 0;

        loop {
            if offset_in_cluster >= self.cluster_size {
                if let Some(next_cluster) = self.find_next_cluster(current_cluster).await? {
                    current_cluster = next_cluster;
                    current_cluster_offset = self.cluster_offset(current_cluster)?;
                    offset_in_cluster = 0;
                } else {
                    break;
                }
            }

            self.storage
                .seek(SeekFrom::Start(
                    current_cluster_offset + offset_in_cluster as u64,
                ))
                .await?;
            offset_in_cluster += 32;

            let mut data = [0u8; 32];
            self.storage.read_exact(&mut data).await?;

            if data[0] == 0x00 {
                break;
            }

            if data[0] == 0xE5 {
                continue;
            }

            let attr = DirEntryAttr::from_bits_truncate(data[11]);
            if attr.contains(DirEntryAttr::LONG_NAME) {
                continue;
            }

            let name: [u8; 11] = data[0..11].try_into().unwrap();
            let start_cluster = u32::from_le_bytes([data[26], data[27], data[20], data[21]]);
            let file_size = u32::from_le_bytes([data[28], data[29], data[30], data[31]]);

            let dir_entry = DirEntry {
                name,
                attr,
                file_size,
                start_cluster,
            };

            if let ControlFlow::Break(()) = f(&dir_entry) {
                break;
            }
        }

        Ok(())
    }

    pub fn file_reader(
        &mut self,
        entry: &DirEntry,
    ) -> Result<FileReader<'_, S>, FsError<S::Error>> {
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
        if cluster < 2 {
            return Err(FsError::InvalidCluster);
        }
        Ok(self.first_cluster_offset + (cluster as u64 - 2) * self.cluster_size as u64)
    }

    async fn find_next_cluster(&mut self, cluster: u32) -> Result<Option<u32>, FsError<S::Error>> {
        let fat_entry_offset = self.fat.offset + cluster as u64 * 4;
        self.storage.seek(SeekFrom::Start(fat_entry_offset)).await?;
        let mut buf = [0u8; 4];
        self.storage.read_exact(&mut buf).await?;
        let next_cluster = u32::from_le_bytes(buf);
        if next_cluster >= 0x0FFFFFF8 {
            return Ok(None);
        }
        let next_cluster = next_cluster & 0x0FFFFFFF;
        if next_cluster < 2 {
            return Err(FsError::InvalidCluster);
        }
        Ok(Some(next_cluster))
    }
}

pub struct RawDirectory {
    start_cluster: u32,
}

#[derive(Clone, Debug)]
pub struct DirEntry {
    name: [u8; 11],
    attr: DirEntryAttr,
    file_size: u32,
    start_cluster: u32,
}

impl DirEntry {
    pub fn name(&self) -> &[u8; 11] {
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
                match self.fs.find_next_cluster(self.current_cluster).await? {
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
    size_in_sectors: u32,
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
