#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "intelhex.h"

static char hex_digit(uint8_t value)
{
    return "0123456789ABCDEF"[value & 0x0f];
}

static void append_hex_byte(std::string &out, uint8_t value)
{
    out.push_back(hex_digit(value >> 4));
    out.push_back(hex_digit(value));
}

static std::string make_record(uint16_t address, uint8_t type, const std::vector<uint8_t> &data)
{
    std::string record(1, ':');
    uint8_t checksum = static_cast<uint8_t>(data.size());

    append_hex_byte(record, static_cast<uint8_t>(data.size()));
    append_hex_byte(record, static_cast<uint8_t>(address >> 8));
    append_hex_byte(record, static_cast<uint8_t>(address));
    append_hex_byte(record, type);
    checksum = static_cast<uint8_t>(checksum + (address >> 8) + address + type);

    for (uint8_t value : data) {
        append_hex_byte(record, value);
        checksum = static_cast<uint8_t>(checksum + value);
    }

    append_hex_byte(record, static_cast<uint8_t>(0U - checksum));
    record += "\r\n";
    return record;
}

static void append_data(std::string &hex, std::map<uint32_t, uint8_t> &expected,
                        uint32_t address, uint32_t size)
{
    for (uint32_t offset = 0; offset < size; offset += 16U) {
        std::vector<uint8_t> data;
        uint32_t record_size = std::min(16U, size - offset);

        for (uint32_t i = 0; i < record_size; ++i) {
            uint8_t value = static_cast<uint8_t>((address + offset + i) ^ 0x5aU);
            data.push_back(value);
            expected[address + offset + i] = value;
        }
        hex += make_record(static_cast<uint16_t>(address + offset), 0x00, data);
    }
}

int main()
{
    std::string hex;
    std::map<uint32_t, uint8_t> expected;
    std::map<uint32_t, uint8_t> decoded;
    uint8_t bin_buffer[256];
    uint32_t input_offset = 0;
    bool eof_seen = false;

    hex += make_record(0, 0x04, {0x02, 0x00});
    append_data(hex, expected, 0x0200f000U, 320U);
    hex += make_record(0, 0x04, {0x02, 0x01});
    append_data(hex, expected, 0x02010100U, 64U);
    hex += make_record(0, 0x01, {});

    reset_hex_parser();
    while (input_offset < hex.size()) {
        uint32_t parse_count = 0;
        uint32_t bin_address = 0;
        uint32_t bin_count = 0;
        uint32_t input_size = std::min<uint32_t>(512U, hex.size() - input_offset);
        hexfile_parse_status_t status = parse_hex_blob(
            reinterpret_cast<const uint8_t *>(hex.data() + input_offset), input_size,
            &parse_count, bin_buffer, sizeof(bin_buffer), &bin_address, &bin_count);

        if ((parse_count == 0U) || (parse_count > input_size)) {
            std::fprintf(stderr, "parser made no progress at offset %u\n", input_offset);
            return 1;
        }
        for (uint32_t i = 0; i < bin_count; ++i) {
            decoded[bin_address + i] = bin_buffer[i];
        }
        input_offset += parse_count;

        if (status == HEX_PARSE_EOF) {
            eof_seen = true;
            break;
        }
        if ((status != HEX_PARSE_OK) && (status != HEX_PARSE_UNALIGNED)) {
            std::fprintf(stderr, "unexpected parser status %d\n", status);
            return 1;
        }
    }

    if (!eof_seen || (decoded != expected)) {
        std::fprintf(stderr, "decoded %zu of %zu expected bytes, eof=%d\n",
                     decoded.size(), expected.size(), eof_seen);
        return 1;
    }

    std::printf("decoded %zu bytes across two extended linear address records\n", decoded.size());
    return 0;
}
