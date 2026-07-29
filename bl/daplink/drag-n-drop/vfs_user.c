#include <string.h>
#include <stdio.h>
#include "vfs_user.h"
#include "vfs_manager.h"
#include "virtual_fs.h"      // 提供 vfs_create_file, vfs_read_only_file 等
#include "flash_decoder.h"   // 提供 flash_decoder_program
#include "error.h" 
#include "util.h"
#include "sys_init.h"

// === 配置区：按需修改 ===
#define DETAILS_CONTENT \
    "DAPLink Compatible Firmware\r\n" \
    "Version: 1.0 (Bare-metal)\r\n" \
    "URL: https://example.com\r\n"

// === 辅助函数 ===

static int is_bin_file(const char *name) {
    const char *ext = strrchr(name, '.');
    if (!ext) return 0;
    return (strcasecmp(ext, ".bin") == 0);
}

static int is_hex_file(const char *name) {
    const char *ext = strrchr(name, '.');
    if (!ext) return 0;
    return (strcasecmp(ext, ".hex") == 0);
}

void vfs_user_file_change_handler(const vfs_filename_t filename, vfs_file_change_t change, vfs_file_t file, vfs_file_t new_file_data) {


    printf("file changed\n");

    if (is_bin_file(filename) || is_hex_file(filename)) {
        printf("file is bin or hex\n");
        return;
    }
    
     if (VFS_FILE_CREATED == change) {
//        vfs_mngr_fs_remount();
        printf("vfs vfs_mngr_fs_remount\n");
     }

}

static uint32_t read_file_details_txt(uint32_t sector_offset, uint8_t *data, uint32_t num_sectors)
{
    return 1;
}

#define VFS_DISK_SIZE (MB(512))
void vfs_user_build_filesystem(void) {
    // Setup the filesystem based on target parameters
    vfs_init("ING", VFS_DISK_SIZE);
    vfs_create_file("DETAILS TXT", read_file_details_txt, 0, 10);
}

void vfs_user_disconnecting(void) {
    printf("vfs disconnect\n");
}
volatile uint8_t reset_flag = 0;
void vfs_user_flash_end(void) {
    reset_flag = 1;
}

void vfs_user_flash_update(void) {
    reset_flag = 0;
}
