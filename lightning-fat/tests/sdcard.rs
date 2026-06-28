use std::ops::ControlFlow;

use embedded_io_adapters::tokio_1::FromTokio;
use embedded_io_async::Read;
use lightning_fat::{DirEntry, DirEntryAttr, FileSystem};

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

    let mut entry: Option<DirEntry> = None;
    fs.iter_dir(&root, |e| {
        if !e.attr().contains(DirEntryAttr::VOLUME_ID) {
            entry = Some(e.clone());
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(())
        }
    })
    .await
    .unwrap();

    let mut reader = fs.file_reader(&entry.unwrap()).unwrap();
    let mut buf = [0u8; 20];
    reader.read_exact(&mut buf).await.unwrap();
    print_buf(0, &buf);
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
