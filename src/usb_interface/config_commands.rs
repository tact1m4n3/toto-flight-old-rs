use crate::{persistent_config as cfg, utils::pack};

/// Get a configuration value and write it to the provided writer.
///
/// Returns `Err(core::fmt::Error)` if the writer runs out of capacity —
/// propagate this to the caller; do not panic.
pub async fn get_config<W: core::fmt::Write>(writer: &mut W, key: &str) -> core::fmt::Result {
    match key {
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
        "gyro_bias" => format_f32_array(writer, cfg::get_gyro_bias().await),
        "accel_bias" => format_f32_array(writer, cfg::get_accel_bias().await),
        "roll_rate_kp" => format_f32(writer, cfg::get_roll_rate_kp().await),
        "roll_rate_ki" => format_f32(writer, cfg::get_roll_rate_ki().await),
        "roll_rate_kff" => format_f32(writer, cfg::get_roll_rate_kff().await),
        "pitch_rate_kp" => format_f32(writer, cfg::get_pitch_rate_kp().await),
        "pitch_rate_ki" => format_f32(writer, cfg::get_pitch_rate_ki().await),
        "pitch_rate_kff" => format_f32(writer, cfg::get_pitch_rate_kff().await),
        "yaw_rate_kp" => format_f32(writer, cfg::get_yaw_rate_kp().await),
        "yaw_rate_ki" => format_f32(writer, cfg::get_yaw_rate_ki().await),
        "yaw_rate_kff" => format_f32(writer, cfg::get_yaw_rate_kff().await),
        "left_elevon_mid_point" => format_i16(writer, cfg::get_left_elevon_mid_point().await),
        "left_elevon_min_us" => format_i16(writer, cfg::get_left_elevon_min_us().await),
        "left_elevon_max_us" => format_i16(writer, cfg::get_left_elevon_max_us().await),
        "left_elevon_mixer_gain" => {
            format_f32_array(writer, cfg::get_left_elevon_mixer_gain().await)
        }
        "right_elevon_mid_point" => format_i16(writer, cfg::get_right_elevon_mid_point().await),
        "right_elevon_min_us" => format_i16(writer, cfg::get_right_elevon_min_us().await),
        "right_elevon_max_us" => format_i16(writer, cfg::get_right_elevon_max_us().await),
        "right_elevon_mixer_gain" => {
            format_f32_array(writer, cfg::get_right_elevon_mixer_gain().await)
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
        "craft_name" => {
            if value.len() >= 15 {
                return Err(Error::InvalidValue);
            }
            cfg::set_craft_name(pack(value)).await;
            Ok(())
        }
        "gyro_bias" => {
            cfg::set_gyro_bias(parse_f32_array(value)?).await;
            Ok(())
        }
        "accel_bias" => {
            cfg::set_accel_bias(parse_f32_array(value)?).await;
            Ok(())
        }
        "roll_rate_kp" => {
            cfg::set_roll_rate_kp(parse_f32(value)?).await;
            Ok(())
        }
        "roll_rate_ki" => {
            cfg::set_roll_rate_ki(parse_f32(value)?).await;
            Ok(())
        }
        "roll_rate_kff" => {
            cfg::set_roll_rate_kff(parse_f32(value)?).await;
            Ok(())
        }
        "pitch_rate_kp" => {
            cfg::set_pitch_rate_kp(parse_f32(value)?).await;
            Ok(())
        }
        "pitch_rate_ki" => {
            cfg::set_pitch_rate_ki(parse_f32(value)?).await;
            Ok(())
        }
        "pitch_rate_kff" => {
            cfg::set_pitch_rate_kff(parse_f32(value)?).await;
            Ok(())
        }
        "yaw_rate_kp" => {
            cfg::set_yaw_rate_kp(parse_f32(value)?).await;
            Ok(())
        }
        "yaw_rate_ki" => {
            cfg::set_yaw_rate_ki(parse_f32(value)?).await;
            Ok(())
        }
        "yaw_rate_kff" => {
            cfg::set_yaw_rate_kff(parse_f32(value)?).await;
            Ok(())
        }
        "left_elevon_mid_point" => {
            cfg::set_left_elevon_mid_point(parse_i16(value)?).await;
            Ok(())
        }
        "left_elevon_min_us" => {
            cfg::set_left_elevon_min_us(parse_i16(value)?).await;
            Ok(())
        }
        "left_elevon_max_us" => {
            cfg::set_left_elevon_max_us(parse_i16(value)?).await;
            Ok(())
        }
        "left_elevon_mixer_gain" => {
            cfg::set_left_elevon_mixer_gain(parse_f32_array(value)?).await;
            Ok(())
        }
        "right_elevon_mid_point" => {
            cfg::set_right_elevon_mid_point(parse_i16(value)?).await;
            Ok(())
        }
        "right_elevon_min_us" => {
            cfg::set_right_elevon_min_us(parse_i16(value)?).await;
            Ok(())
        }
        "right_elevon_max_us" => {
            cfg::set_right_elevon_max_us(parse_i16(value)?).await;
            Ok(())
        }
        "right_elevon_mixer_gain" => {
            cfg::set_right_elevon_mixer_gain(parse_f32_array(value)?).await;
            Ok(())
        }
        _ => Err(Error::UnknownKey),
    }
}

// ── Parsing helpers ───────────────────────────────────────────────────────────

fn parse_f32(s: &str) -> Result<f32, Error> {
    s.trim().parse::<f32>().map_err(|_| Error::InvalidValue)
}

fn parse_i16(s: &str) -> Result<i16, Error> {
    s.trim().parse::<i16>().map_err(|_| Error::InvalidValue)
}

/// Parse a bracket-delimited, comma-separated array of exactly `N` `f32` values.
///
/// Accepts `[1.0, 2.0, 3.0]` or `1.0, 2.0, 3.0` (brackets optional).
/// Returns `Err(Error::InvalidValue)` if the element count ≠ N or any token
/// fails to parse.
fn parse_f32_array<const N: usize>(s: &str) -> Result<[f32; N], Error> {
    let s = s.trim().trim_start_matches('[').trim_end_matches(']');

    let mut values = [0.0f32; N];
    let mut count = 0;

    for token in s.split(',') {
        let token = token.trim();
        if token.is_empty() {
            continue;
        }
        if count >= N {
            return Err(Error::InvalidValue);
        }
        values[count] = token.parse::<f32>().map_err(|_| Error::InvalidValue)?;
        count += 1;
    }

    if count != N {
        return Err(Error::InvalidValue);
    }

    Ok(values)
}

// ── Formatting helpers ────────────────────────────────────────────────────────

/// Write an `Option<f32>`, returning `Err(fmt::Error)` on buffer overflow.
fn format_f32<W: core::fmt::Write>(writer: &mut W, value: Option<f32>) -> core::fmt::Result {
    match value {
        Some(v) => format_float_fmt(writer, v),
        None => write!(writer, "error: not set"),
    }
}

/// Write an `Option<i16>`, returning `Err(fmt::Error)` on buffer overflow.
fn format_i16<W: core::fmt::Write>(writer: &mut W, value: Option<i16>) -> core::fmt::Result {
    match value {
        Some(v) => write!(writer, "{}", v),
        None => write!(writer, "error: not set"),
    }
}

/// Write a bracket-delimited, comma-separated array of exactly `N` `f32` values.
///
/// Returns `Err(fmt::Error)` on buffer overflow — the caller must propagate this.
fn format_f32_array<W: core::fmt::Write, const N: usize>(
    writer: &mut W,
    value: Option<[f32; N]>,
) -> core::fmt::Result {
    let Some(arr) = value else {
        return write!(writer, "error: not set");
    };
    write!(writer, "[")?;
    for (i, v) in arr.iter().enumerate() {
        if i > 0 {
            write!(writer, ", ")?;
        }
        format_float_fmt(writer, *v)?;
    }
    write!(writer, "]")
}

/// Format a float suppressing unnecessary trailing decimal places.
///
/// Integers within the safe i64 range are written without a decimal point.
/// All `write!` calls return `Err(fmt::Error)` on overflow — never panics.
fn format_float_fmt<W: core::fmt::Write>(writer: &mut W, value: f32) -> core::fmt::Result {
    let truncated = value as i64 as f32;
    if (value - truncated).abs() < 1e-6 && value.abs() < 1e9 {
        write!(writer, "{}", value as i64)
    } else {
        write!(writer, "{}", value)
    }
}
