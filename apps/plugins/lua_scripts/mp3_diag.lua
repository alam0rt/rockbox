-- Dump the current track metadata and inspect bytes around the MP3 frame.
-- Run this while the failing track is the current playing track.

require("rbsettings")

local entry = rb.metadata.mp3_entry
local current = "audio_current_track"
local report = {}

local function value(field)
    return rb.settings.read(current, field)
end

local function text(v)
    if v == nil then
        return "<nil>"
    end
    return tostring(v)
end

local function add(fmt, ...)
    report[#report + 1] = string.format(fmt, ...)
end

local function hex(data)
    if not data then
        return "<read failed>"
    end

    local bytes = {}
    for i = 1, #data do
        bytes[#bytes + 1] = string.format("%02x", string.byte(data, i))
    end
    return table.concat(bytes, " ")
end

local path = value(entry.path)
if not path or path == "" then
    rb.splash(rb.HZ * 3, "No current track")
    return
end

add("path: %s", path)
add("length_ms: %s", text(value(entry.length)))
add("filesize: %s", text(value(entry.filesize)))
add("id3v2len: %s", text(value(entry.id3v2len)))
add("id3v1len: %s", text(value(entry.id3v1len)))
add("first_frame: %s", text(value(entry.first_frame_offset)))
add("bitrate: %s", text(value(entry.bitrate)))
add("frequency: %s", text(value(entry.frequency)))
add("layer: %s", text(value(entry.layer)))
add("codectype: %s", text(value(entry.codectype)))
add("frame_count: %s", text(value(entry.frame_count)))
add("vbr: %s", text(value(entry.vbr)))
add("elapsed_ms: %s", text(value(entry.elapsed)))

local file = io.open(path, "r")
if not file then
    add("file_open: failed")
else
    local file_size = file:seek("end")
    add("disk_filesize: %s", text(file_size))

    local function read_at(pos, count)
        if not pos or pos < 0 or pos > file_size then
            return "<invalid offset>"
        end
        if not file:seek("set", pos) then
            return "<seek failed>"
        end
        return hex(file:read(count))
    end

    local id3len = value(entry.id3v2len) or 0
    local first_frame = value(entry.first_frame_offset) or 0
    add("bytes_at_0: %s", read_at(0, 16))
    add("bytes_at_id3v2: %s", read_at(id3len, 16))
    add("bytes_at_first: %s", read_at(first_frame, 16))

    local scan_start = id3len
    local scan_limit = math.min(file_size, scan_start + 0x100000)
    local scan_pos = scan_start
    local carry = ""
    local found

    while scan_pos < scan_limit and not found do
        local count = math.min(512, scan_limit - scan_pos)
        if not file:seek("set", scan_pos) then
            break
        end

        local chunk = file:read(count)
        if not chunk or #chunk == 0 then
            break
        end

        local data = carry .. chunk
        local data_start = scan_pos - #carry
        for i = 1, #data - 3 do
            local b1, b2 = string.byte(data, i, i + 1)
            if b1 == 0xff and b2 >= 0xe0 then
                found = data_start + i - 1
                break
            end
        end

        carry = data:sub(math.max(1, #data - 2))
        scan_pos = scan_pos + #chunk
    end

    add("candidate_frame: %s", text(found))
    if found then
        add("bytes_at_candidate: %s", read_at(found, 16))
    end
    file:close()
end

local out = io.open("/mp3diag.txt", "w+")
if not out then
    rb.splash(rb.HZ * 3, "Cannot write /mp3diag.txt")
    return
end

out:write(table.concat(report, "\n"), "\n")
out:close()
rb.splash(rb.HZ * 3, "Wrote /mp3diag.txt")
