use embedded_io_adapters::tokio_1::FromTokio;
use embedded_io_async::{Read, ReadExactError, Seek, SeekFrom, Write};

const SECTOR_SIZE: u32 = 512;

#[tokio::test]
pub async fn test_sdcard() {
    //
    //for entry in fs.list(dir) {
    //
    //}
    //fs.create_file(dir, name, Options {
    //    size: usize,
    //});
    let mut file = FromTokio::new(
        tokio::fs::File::open("tests/sdcard_fat32.img")
            .await
            .unwrap(),
    );
    let mut fs = FileSystem::new(&mut file).await.unwrap();
    let root = fs.root_dir();
    fs.iter_dir(&root, |entry| {
        println!(
            "Entry: name={:?}, is_directory={}, size={}",
            entry.name(),
            entry.is_directory(),
            entry.size()
        );
    })
    .await
    .unwrap();
}

trait ReadWriteSeek: Read + Write + Seek {}
impl<T: Read + Write + Seek> ReadWriteSeek for T {}

pub struct FileSystem<S: ReadWriteSeek> {
    storage: S,
    boot_sector: BootSector,
    fat: FileAllocationTable,
    first_cluster_offset: u64,
    cluster_size: u32,
}

impl<S: ReadWriteSeek> FileSystem<S> {
    pub async fn new(mut storage: S) -> Result<Self, ReadExactError<S::Error>> {
        const MBR_OFFSET_PARTITION_1_ENTRY: u64 = 446;
        storage
            .seek(SeekFrom::Start(MBR_OFFSET_PARTITION_1_ENTRY))
            .await?;
        let partition_entry = {
            let mut data = [0u8; 16];
            storage.read_exact(&mut data).await?;
            PartitionEntry {
                typ: data[4],
                starting_lba: u32::from_le_bytes(data[8..12].try_into().unwrap()),
                size_in_sectors: u32::from_le_bytes(data[12..16].try_into().unwrap()),
            }
        };
        assert!(partition_entry.typ == 0x0C);

        let partition_offset = partition_entry.starting_lba as u64 * SECTOR_SIZE as u64;
        storage.seek(SeekFrom::Start(partition_offset)).await?;

        let boot_sector = {
            let mut data = [0u8; 50];
            storage.read_exact(&mut data).await?;
            BootSector {
                bytes_per_sector: u16::from_le_bytes(data[11..13].try_into().unwrap()),
                sectors_per_cluster: data[13],
                reserved_sectors: u16::from_le_bytes(data[14..16].try_into().unwrap()),
                number_of_fats: data[16],
                total_sectors: u32::from_le_bytes(data[32..36].try_into().unwrap()),
                sectors_per_fat: u32::from_le_bytes(data[36..40].try_into().unwrap()),
                root_directory_cluster: u32::from_le_bytes(data[44..48].try_into().unwrap()),
                fs_info_sector: u16::from_le_bytes(data[48..50].try_into().unwrap()),
            }
        };
        assert!(boot_sector.bytes_per_sector as u32 == SECTOR_SIZE);

        // let fsinfo_sector_offset =
        //     partition_offset + boot_sector.fs_info_sector as u64 * SECTOR_SIZE;

        let fat = FileAllocationTable {
            offset: partition_offset + boot_sector.reserved_sectors as u64 * SECTOR_SIZE as u64,
            entries_count: boot_sector.sectors_per_fat * (boot_sector.bytes_per_sector as u32 / 4), // FAT32
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

    pub async fn iter_dir<F: FnMut(&DirEntry)>(
        &mut self,
        dir: &RawDirectory,
        mut f: F,
    ) -> Result<(), ReadExactError<S::Error>> {
        let mut current_cluster = dir.start_cluster;
        let mut current_cluster_offset = self.cluster_offset(current_cluster);
        let mut offset_in_cluster = 0;

        println!("{}", current_cluster);

        loop {
            if offset_in_cluster >= self.cluster_size {
                if let Some(next_cluster) = self.find_next_cluster(current_cluster).await? {
                    current_cluster = next_cluster;
                    current_cluster_offset = self.cluster_offset(current_cluster);
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

            let mut data = [0u8; 32];
            self.storage.read_exact(&mut data).await?;

            if data[0] == 0x00 {
                break;
            }

            // if the entry is free (deleted)
            if data[0] == 0xE5 {
                continue;
            }

            let attr = DirEntryAttr::from_bits_truncate(data[11]);
            if attr.contains(DirEntryAttr::LONG_NAME) {
                continue;
            }

            let name: [u8; 11] = data[0..11].try_into().unwrap();
            let file_size = u32::from_le_bytes(data[28..][..4].try_into().unwrap());

            match DirEntryKind::from_storage(&mut self.storage).await? {
                DirEntryKind::Available(dir_entry) => {}
                DirEntryKind::Skip => (),
                DirEntryKind::EndOfDirectory => {
                    break;
                }
            }

            f(&dir_entry);
            offset_in_cluster += 32;
        }

        Ok(())
    }

    fn cluster_offset(&self, cluster: u32) -> u64 {
        assert!(cluster >= 2);
        self.first_cluster_offset + (cluster as u64 - 2) * self.cluster_size as u64
    }

    async fn find_next_cluster(
        &mut self,
        cluster: u32,
    ) -> Result<Option<u32>, ReadExactError<S::Error>> {
        let fat_entry_offset = self.fat.offset + cluster as u64 * 4;
        self.storage.seek(SeekFrom::Start(fat_entry_offset)).await?;
        let mut buf = [0u8; 4];
        self.storage.read_exact(&mut buf).await?;
        let next_cluster = u32::from_le_bytes(buf);
        if next_cluster >= 0x0FFFFFF8 {
            // end of cluster chain
            return Ok(None);
        }
        Ok(Some(next_cluster & 0x0FFFFFFF))
    }
}

pub struct RawDirectory {
    start_cluster: u32,
}

enum DirEntryKind {
    Available(DirEntry),
    Skip,
    EndOfDirectory,
}

impl DirEntryKind {
    async fn from_storage<S: Read>(
        storage: &mut S,
    ) -> Result<DirEntryKind, ReadExactError<S::Error>> {
    }
}

#[derive(Debug)]
pub struct DirEntry {
    name: [u8; 11],
    attr: DirEntryAttr,
    file_size: u32,
}

impl DirEntry {
    pub fn name(&self) -> &[u8; 11] {
        &self.name
    }

    pub fn is_directory(&self) -> bool {
        self.attr.contains(DirEntryAttr::DIRECTORY)
    }

    pub fn size(&self) -> u32 {
        self.file_size
    }
}

bitflags::bitflags! {
    #[derive(Debug)]
    struct DirEntryAttr: u8 {
        const READ_ONLY = 0x01;
        const HIDDEN = 0x02;
        const SYSTEM = 0x04;
        const VOLUME_ID = 0x08;
        const DIRECTORY = 0x10;
        const ARCHIVE = 0x20;
        const LONG_NAME = Self::READ_ONLY.bits() | Self::HIDDEN.bits() | Self::SYSTEM.bits() | Self::VOLUME_ID.bits();
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

pub fn print_buf(start_addr: u64, buf: &[u8]) {
    for (i, chunk) in buf.chunks(16).enumerate() {
        print!("{:016X}: ", start_addr + i as u64 * 16);
        for byte in chunk {
            print!("{:02X} ", byte);
        }
        println!();
    }
}

// trait ReadExt: Read {
//     async fn read_u8(&mut self) -> Result<u8, ReadExactError<Self::Error>> {
//         let mut buf = [0u8; 1];
//         self.read_exact(&mut buf).await?;
//         Ok(buf[0])
//     }
//
//     async fn read_u16_le(&mut self) -> Result<u16, ReadExactError<Self::Error>> {
//         let mut buf = [0u8; 2];
//         self.read_exact(&mut buf).await?;
//         Ok(u16::from_le_bytes(buf))
//     }
//
//     async fn read_u32_le(&mut self) -> Result<u32, ReadExactError<Self::Error>> {
//         let mut buf = [0u8; 4];
//         self.read_exact(&mut buf).await?;
//         Ok(u32::from_le_bytes(buf))
//     }
//
//     async fn read_u16_be(&mut self) -> Result<u16, ReadExactError<Self::Error>> {
//         let mut buf = [0u8; 2];
//         self.read_exact(&mut buf).await?;
//         Ok(u16::from_be_bytes(buf))
//     }
//
//     async fn read_u32_be(&mut self) -> Result<u32, ReadExactError<Self::Error>> {
//         let mut buf = [0u8; 4];
//         self.read_exact(&mut buf).await?;
//         Ok(u32::from_be_bytes(buf))
//     }
// }
