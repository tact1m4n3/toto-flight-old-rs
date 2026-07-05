use embedded_io_adapters::tokio_1::FromTokio;
use embedded_io_async::{Read, Write};
use lightning_fat::{FileSystem, ShortFileName};

#[tokio::test]
pub async fn test_sdcard() {
    let mut file = FromTokio::new(
        tokio::fs::File::options()
            .write(true)
            .read(true)
            .open("tests/sdcard_fat32.img")
            .await
            .unwrap(),
    );
    let mut fs = FileSystem::new(&mut file).await.unwrap();
    let root = fs.root_dir();

    // for i in 0..128 {
    //     let name = format!("{}.txt", i);
    //     let file = fs.create_file(&root, ShortFileName::new(name.as_bytes()).unwrap()).await.unwrap();
    // }
    // let file = fs
    //     .open_file(root, ShortFileName::new(b"hello.txt").unwrap())
    //     .await
    //     .unwrap();

    let mut alloc_file = fs
        .get_or_create_cluster_allocator_file(10, 1024 * 1024..=10 * 1024 * 1024)
        .await
        .unwrap();
    // let mut file = fs
    //     .create_file(&root, ShortFileName::new(b"LOG.TXT").unwrap())
    //     .await
    //     .unwrap();
    // let mut writer = fs.file_writer(&mut file, &mut alloc_file).unwrap();
    // for _ in 0..1000 {
    //     writer
    //         .write_all(b"Hello world! This is a test of the file writer.\n")
    //         .await
    //         .unwrap();
    // }
    // writer.close().await.unwrap();
    // let file = fs
    //     .open_file(&root, &ShortFileName::new(b"LOG.TXT").unwrap())
    //     .await
    //     .unwrap();
    // let mut reader = fs.file_reader(&file).unwrap();
    // let mut buf: [u8; 64] = [0; 64];
    //
    // loop {
    //     let n = reader.read(&mut buf).await.unwrap();
    //     if n == 0 {
    //         break;
    //     }
    //     print!("{}", std::str::from_utf8(&buf[..n]).unwrap());
    // }

    let mut it = fs.iter_dir(&root);
    while let Some(dir_entry) = it.next().await.unwrap() {
        println!("Entry: {} {:X}", dir_entry.name(), dir_entry.size());
        println!("{:?}", dir_entry);
    }

    // Alloc file is guaranteed to contain consecutive clusters of memory.
    // start_cluster = first cluster in the array
    // size = cluster_count * cluster size
    // Each cluster in this array begins with 0xaa_aa_aa_aa, ie: not used. Can be used for repairing
    // the file in an unfortunate power loss.
    // This method should do a validity check of an already existent alloc file and if it is not
    // valid, try to repair it.
    // Repairing procedure: Look at start_cluster and check the uninitialized signature. If it is
    // uninitialized, everything is ok. If not, look for the first uninitialized signature, which is
    // the chunk after the last chunk used by the previous file writer. We can mark the previous
    // chunk as end of chain, update the alloc_file entry and the previous file entry (how do we get
    // this one?).
    // let alloc_file = fs.get_or_create_cluster_allocator_file(10, 1024 * 1024).await.unwrap();

    // let mut it = fs.iter_dir(&root);
    // while let Some(dir_entry) = it.next().await.unwrap() {
    //     println!("Entry: {} {:X}", dir_entry.name(), dir_entry.size());
    // }

    // let log_file = fs.create_file(&root, ShortFileName::new(b"LOG_0.TXT").unwrap()).await.unwrap();

    // let file_writer = fs.file_writer(&mut log_file, &mut alloc_file);

    // Borrows clusters from alloc file beginning with start_cluster. Should require no actual
    // fat lookup.
    // file_writer.write().await;

    // Updates alloc_file dir entry.
    // file_writer.close().await;
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
