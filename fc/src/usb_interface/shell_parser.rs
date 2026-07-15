use core::fmt::Write;

use heapless::String;

use crate::config;

pub struct ShellParser<const N: usize> {
    line_buf: [u8; N],
    line_buf_end: usize,
}

impl<const N: usize> ShellParser<N> {
    pub const fn new() -> Self {
        Self {
            line_buf: [0; N],
            line_buf_end: 0,
        }
    }

    pub fn parse<'b>(
        &mut self,
        data: &'b [u8],
    ) -> Option<(Result<ShellCommand, ShellParserError>, &'b [u8])> {
        let mut line_iter = data.splitn(2, |&b| b == b'\n');
        let line_end = line_iter.next().unwrap(); // there is at least one item

        if self.line_buf_end + line_end.len() > self.line_buf.len() {
            // line too long
            self.line_buf_end = 0;
            return Some((
                Err(ShellParserError::LineTooLong),
                line_iter.next().unwrap_or(&[]),
            ));
        }

        self.line_buf[self.line_buf_end..][..line_end.len()].copy_from_slice(line_end);
        self.line_buf_end += line_end.len();

        let remaining = line_iter.next()?;

        // we have a line! let's parse it
        let line = &self.line_buf[..self.line_buf_end];
        self.line_buf_end = 0;

        let Ok(line) = core::str::from_utf8(line) else {
            return Some((Err(ShellParserError::InvalidUtf8Line), remaining));
        };

        Some((Self::parse_line(line), remaining))
    }

    fn parse_line(line: &str) -> Result<ShellCommand, ShellParserError> {
        let mut word_iter = line.split_whitespace();
        match word_iter.next() {
            Some("reboot") => Ok(ShellCommand::Reboot),
            Some("calibrate") => {
                let calibration_type = word_iter.next().ok_or(ShellParserError::InvalidCommand)?;
                match calibration_type {
                    "imu" => Ok(ShellCommand::Calibrate(CalibrationType::ImuEasyCalibration)),
                    _ => Err(ShellParserError::InvalidCommand),
                }
            }
            Some("set") => {
                let config_name = word_iter.next().ok_or(ShellParserError::InvalidCommand)?;
                match config_name {
                    "craft_name" => {
                        let craft_name =
                            word_iter.next().ok_or(ShellParserError::InvalidCommand)?;
                        let mut craft_name_string = String::new();
                        craft_name_string
                            .write_str(craft_name)
                            .map_err(|_| ShellParserError::InvalidCommand)?;
                        Ok(ShellCommand::ConfigSet(ConfigSet::CraftName(
                            craft_name_string,
                        )))
                    }
                    _ => Err(ShellParserError::InvalidCommand),
                }
            }
            Some("get") => {
                let config_name = word_iter.next().ok_or(ShellParserError::InvalidCommand)?;
                match config_name {
                    "craft_name" => Ok(ShellCommand::ConfigGet(ConfigGet::CraftName)),
                    _ => Err(ShellParserError::InvalidCommand),
                }
            }
            Some("save") => Ok(ShellCommand::ConfigSave),
            Some(_) | None => Err(ShellParserError::InvalidCommand),
        }
    }
}

pub enum ShellCommand {
    Reboot,
    Calibrate(CalibrationType),
    ConfigSet(ConfigSet),
    ConfigGet(ConfigGet),
    ConfigSave,
}

pub enum CalibrationType {
    ImuEasyCalibration,
}

#[derive(Debug)]
pub enum ConfigSet {
    CraftName(String<16>),
}

pub enum ConfigGet {
    CraftName,
}

#[derive(Debug)]
pub enum ShellParserError {
    LineTooLong,
    InvalidUtf8Line,
    InvalidCommand,
}

impl core::fmt::Display for ShellParserError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ShellParserError::LineTooLong => write!(f, "line too long"),
            ShellParserError::InvalidUtf8Line => write!(f, "invalid utf8 line"),
            ShellParserError::InvalidCommand => write!(f, "invalid command"),
        }
    }
}
