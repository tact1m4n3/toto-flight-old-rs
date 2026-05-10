use crate::{persistent_config as cfg, utils::pack};

/// Get a configuration value and write it to the provided writer.
///
/// Returns `Err(core::fmt::Error)` if the writer runs out of capacity —
/// propagate this to the caller; do not panic.
pub async fn get_config<W: core::fmt::Write>(writer: &mut W, key: &str) -> core::fmt::Result {
    match key {
        "board_forward_angle" => format_num(writer, cfg::get_board_forward_angle().await),
        "gyro_bias" => format_num_array(writer, cfg::get_gyro_bias().await),
        "imu_orientation" => format_num_array(writer, cfg::get_imu_orientation().await),
        "craft_name" => {
            if let Some(value) = cfg::get_craft_name().await {
                let len = value.iter().position(|&b| b == 0).unwrap_or(16);
                match core::str::from_utf8(&value[..len]) {
                    Ok(s) => write!(writer, "{}", s),
                    Err(_) => write!(writer, "error: invalid utf8"),
                }
            } else {
                write!(writer, "error: not set")
            }
        }
        "roll_rate_kp" => format_num(writer, cfg::get_roll_rate_kp().await),
        "roll_rate_ki" => format_num(writer, cfg::get_roll_rate_ki().await),
        "roll_rate_kff" => format_num(writer, cfg::get_roll_rate_kff().await),
        "pitch_rate_kp" => format_num(writer, cfg::get_pitch_rate_kp().await),
        "pitch_rate_ki" => format_num(writer, cfg::get_pitch_rate_ki().await),
        "pitch_rate_kff" => format_num(writer, cfg::get_pitch_rate_kff().await),
        "yaw_rate_kp" => format_num(writer, cfg::get_yaw_rate_kp().await),
        "yaw_rate_ki" => format_num(writer, cfg::get_yaw_rate_ki().await),
        "yaw_rate_kff" => format_num(writer, cfg::get_yaw_rate_kff().await),
        "left_elevon_mid_point" => format_num(writer, cfg::get_left_elevon_mid_point().await),
        "left_elevon_range_us" => format_num_array(writer, cfg::get_left_elevon_range_us().await),
        "left_elevon_mixer_gain" => {
            format_num_array(writer, cfg::get_left_elevon_mixer_gain().await)
        }
        "right_elevon_mid_point" => format_num(writer, cfg::get_right_elevon_mid_point().await),
        "right_elevon_range_us" => format_num_array(writer, cfg::get_right_elevon_range_us().await),
        "right_elevon_mixer_gain" => {
            format_num_array(writer, cfg::get_right_elevon_mixer_gain().await)
        }
        _ => write!(writer, "error: unknown key '{key}'"),
    }
}

/// Set a configuration value parsed from `value`, writing the outcome to `writer`.
///
/// Writes `"ok"` on success, or a descriptive error message on failure.
/// Returns `Err(core::fmt::Error)` only if the writer runs out of capacity.
pub async fn set_config<W: core::fmt::Write>(
    writer: &mut W,
    key: &str,
    value: &str,
) -> core::fmt::Result {
    match inner_set_config(key, value).await {
        Ok(()) => write!(writer, "ok"),
        Err(Error::UnknownKey) => write!(writer, "error: unknown key '{key}'"),
        Err(Error::InvalidValue) => {
            write!(writer, "error: invalid value '{}' for key '{}'", value, key)
        }
    }
}

#[derive(Debug)]
enum Error {
    UnknownKey,
    InvalidValue,
}

async fn inner_set_config(key: &str, value: &str) -> Result<(), Error> {
    match key {
        "board_forward_angle" => {
            cfg::set_board_forward_angle(parse_num(value)?).await;
            Ok(())
        }
        "gyro_bias" => {
            cfg::set_gyro_bias(parse_num_array(value)?).await;
            Ok(())
        }
        "imu_orientation" => {
            cfg::set_imu_orientation(parse_num_array(value)?).await;
            Ok(())
        }
        "craft_name" => {
            if value.len() >= 15 {
                return Err(Error::InvalidValue);
            }
            cfg::set_craft_name(pack(value)).await;
            Ok(())
        }
        "roll_rate_kp" => {
            cfg::set_roll_rate_kp(parse_num(value)?).await;
            Ok(())
        }
        "roll_rate_ki" => {
            cfg::set_roll_rate_ki(parse_num(value)?).await;
            Ok(())
        }
        "roll_rate_kff" => {
            cfg::set_roll_rate_kff(parse_num(value)?).await;
            Ok(())
        }
        "pitch_rate_kp" => {
            cfg::set_pitch_rate_kp(parse_num(value)?).await;
            Ok(())
        }
        "pitch_rate_ki" => {
            cfg::set_pitch_rate_ki(parse_num(value)?).await;
            Ok(())
        }
        "pitch_rate_kff" => {
            cfg::set_pitch_rate_kff(parse_num(value)?).await;
            Ok(())
        }
        "yaw_rate_kp" => {
            cfg::set_yaw_rate_kp(parse_num(value)?).await;
            Ok(())
        }
        "yaw_rate_ki" => {
            cfg::set_yaw_rate_ki(parse_num(value)?).await;
            Ok(())
        }
        "yaw_rate_kff" => {
            cfg::set_yaw_rate_kff(parse_num(value)?).await;
            Ok(())
        }
        "left_elevon_mid_point" => {
            cfg::set_left_elevon_mid_point(parse_num(value)?).await;
            Ok(())
        }
        "left_elevon_range_us" => {
            cfg::set_left_elevon_range_us(parse_num_array(value)?).await;
            Ok(())
        }
        "left_elevon_mixer_gain" => {
            cfg::set_left_elevon_mixer_gain(parse_num_array(value)?).await;
            Ok(())
        }
        "right_elevon_mid_point" => {
            cfg::set_right_elevon_mid_point(parse_num(value)?).await;
            Ok(())
        }
        "right_elevon_range_us" => {
            cfg::set_right_elevon_range_us(parse_num_array(value)?).await;
            Ok(())
        }
        "right_elevon_mixer_gain" => {
            cfg::set_right_elevon_mixer_gain(parse_num_array(value)?).await;
            Ok(())
        }
        _ => Err(Error::UnknownKey),
    }
}

fn parse_num<T: Copy + num_traits::Num>(s: &str) -> Result<T, Error> {
    T::from_str_radix(s.trim(), 10).map_err(|_| Error::InvalidValue)
}

fn parse_num_array<const N: usize, T: Copy + num_traits::Num>(s: &str) -> Result<[T; N], Error> {
    let s = s.trim().trim_start_matches('[').trim_end_matches(']');

    let mut values = [T::zero(); N];
    let mut count = 0;

    for token in s.split(',') {
        let token = token.trim();
        if token.is_empty() {
            continue;
        }
        if count >= N {
            return Err(Error::InvalidValue);
        }
        values[count] = T::from_str_radix(token, 10).map_err(|_| Error::InvalidValue)?;
        count += 1;
    }

    if count != N {
        return Err(Error::InvalidValue);
    }

    Ok(values)
}

fn format_num<W: core::fmt::Write, T: core::fmt::Display>(
    writer: &mut W,
    value: Option<T>,
) -> core::fmt::Result {
    match value {
        Some(v) => write!(writer, "{}", v),
        None => write!(writer, "not set"),
    }
}

fn format_num_array<W: core::fmt::Write, T: core::fmt::Display, const N: usize>(
    writer: &mut W,
    value: Option<[T; N]>,
) -> core::fmt::Result {
    let Some(arr) = value else {
        return write!(writer, "not set");
    };
    write!(writer, "[")?;
    for (i, v) in arr.iter().enumerate() {
        if i > 0 {
            write!(writer, ", ")?;
        }
        write!(writer, "{}", v)?;
    }
    write!(writer, "]")
}
