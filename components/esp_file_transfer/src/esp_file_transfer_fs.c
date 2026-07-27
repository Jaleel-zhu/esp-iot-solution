/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_file_transfer_internal.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include "esp_log.h"

static const char *TAG = "esp_file_transfer";

static char *join_path(const char *dir, const char *name)
{
    size_t dir_len = strlen(dir);
    size_t name_len = strlen(name);
    bool slash = dir_len > 0 && dir[dir_len - 1] == '/';
    if (dir_len > SIZE_MAX - name_len - 2) {
        return NULL;
    }
    char *path = malloc(dir_len + name_len + (slash ? 1 : 2));
    if (!path) {
        return NULL;
    }
    snprintf(path, dir_len + name_len + (slash ? 1 : 2), slash ? "%s%s" : "%s/%s", dir, name);
    return path;
}

static bool utf8_valid(const uint8_t *text, size_t len)
{
    for (size_t i = 0; i < len;) {
        uint8_t c = text[i];
        if (c < 0x80) {
            ++i;
            continue;
        }
        size_t count;
        uint32_t codepoint;
        if (c >= 0xc2 && c <= 0xdf) {
            count = 1;
            codepoint = c & 0x1f;
        } else if (c >= 0xe0 && c <= 0xef) {
            count = 2;
            codepoint = c & 0x0f;
        } else if (c >= 0xf0 && c <= 0xf4) {
            count = 3;
            codepoint = c & 0x07;
        } else {
            return false;
        }
        if (i + count >= len) {
            return false;
        }
        for (size_t j = 1; j <= count; ++j) {
            if ((text[i + j] & 0xc0) != 0x80) {
                return false;
            }
            codepoint = (codepoint << 6) | (text[i + j] & 0x3f);
        }
        if ((count == 2 && codepoint < 0x800) ||
                (count == 3 && codepoint < 0x10000) ||
                (codepoint >= 0xd800 && codepoint <= 0xdfff) ||
                codepoint > 0x10ffff) {
            return false;
        }
        i += count + 1;
    }
    return true;
}

bool ft_fs_name_valid(const char *name)
{
    if (!name) {
        return false;
    }
    size_t len = strnlen(name, ESP_FT_FILE_NAME_MAX_LEN + 1);
    if (len == 0 || len > ESP_FT_FILE_NAME_MAX_LEN ||
            (len == 1 && name[0] == '.') || name[0] == '/' || name[0] == '\\' ||
            (len >= 2 && name[1] == ':' &&
             ((name[0] >= 'A' && name[0] <= 'Z') || (name[0] >= 'a' && name[0] <= 'z')))) {
        return false;
    }
    for (size_t i = 0; i < len; ++i) {
        uint8_t c = (uint8_t)name[i];
        if (c < 0x20 || c == 0x7f || c == '/' || c == '\\' ||
                (c == '.' && i + 1 < len && name[i + 1] == '.')) {
            return false;
        }
    }
    return utf8_valid((const uint8_t *)name, len);
}

const char *ft_fs_basename(const char *path)
{
    if (!path) {
        return NULL;
    }
    const char *slash = strrchr(path, '/');
    const char *backslash = strrchr(path, '\\');
    const char *base = slash ? slash + 1 : path;
    if (backslash && backslash + 1 > base) {
        base = backslash + 1;
    }
    return base;
}

esp_err_t ft_fs_prepare(const char *recv_dir)
{
    if (!recv_dir || recv_dir[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    struct stat info;
    if (stat(recv_dir, &info) != 0 || !S_ISDIR(info.st_mode) || access(recv_dir, W_OK) != 0) {
        return ESP_FT_ERR_OPEN_FAILED;
    }
    char *temp_dir = join_path(recv_dir, ".ft_tmp");
    if (!temp_dir) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t result = ESP_OK;
    if (mkdir(temp_dir, 0700) != 0 && errno != EEXIST) {
        result = ESP_FT_ERR_OPEN_FAILED;
    } else if (stat(temp_dir, &info) != 0 || !S_ISDIR(info.st_mode)) {
        result = ESP_FT_ERR_OPEN_FAILED;
    }
    free(temp_dir);
    return result;
}

void ft_fs_cleanup_parts(const char *recv_dir)
{
    char *temp_dir = join_path(recv_dir, ".ft_tmp");
    if (!temp_dir) {
        return;
    }
    DIR *dir = opendir(temp_dir);
    if (!dir) {
        free(temp_dir);
        return;
    }
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        size_t len = strlen(entry->d_name);
        if (len < 5 || strcmp(entry->d_name + len - 5, ".part") != 0) {
            continue;
        }
        char *path = join_path(temp_dir, entry->d_name);
        if (path) {
            if (unlink(path) != 0 && errno != ENOENT) {
                ESP_LOGW(TAG, "failed to remove stale part %s: errno=%d", path, errno);
            }
            free(path);
        }
    }
    closedir(dir);
    free(temp_dir);
}

esp_err_t ft_fs_source_info(const char *path, uint64_t *size)
{
    if (!path || !size) {
        return ESP_ERR_INVALID_ARG;
    }
    struct stat info;
    if (stat(path, &info) != 0) {
        return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FT_ERR_OPEN_FAILED;
    }
    if (!S_ISREG(info.st_mode) || info.st_size < 0) {
        return ESP_FT_ERR_OPEN_FAILED;
    }
    *size = (uint64_t)info.st_size;
    return ESP_OK;
}

esp_err_t ft_fs_open_source(const char *path, FILE **file)
{
    if (!path || !file) {
        return ESP_ERR_INVALID_ARG;
    }
    *file = fopen(path, "rb");
    if (!*file) {
        return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FT_ERR_OPEN_FAILED;
    }
    return ESP_OK;
}

esp_err_t ft_fs_create_temp(const char *recv_dir, uint32_t transfer_id, const char *file_name,
                            char **temp_path, FILE **file)
{
    if (!recv_dir || transfer_id == 0 || !ft_fs_name_valid(file_name) || !temp_path || !file) {
        return ESP_ERR_INVALID_ARG;
    }
    char *temp_dir = join_path(recv_dir, ".ft_tmp");
    if (!temp_dir) {
        return ESP_ERR_NO_MEM;
    }
    size_t name_len = strlen(file_name) + 8 + 1 + 5 + 1;
    char *part_name = malloc(name_len);
    if (!part_name) {
        free(temp_dir);
        return ESP_ERR_NO_MEM;
    }
    snprintf(part_name, name_len, "%08" PRIx32 "_%s.part", transfer_id, file_name);
    char *path = join_path(temp_dir, part_name);
    free(part_name);
    free(temp_dir);
    if (!path) {
        return ESP_ERR_NO_MEM;
    }
    int fd = open(path, O_RDWR | O_CREAT | O_EXCL, 0600);
    if (fd < 0) {
        esp_err_t err = errno == ENOSPC ? ESP_ERR_NO_MEM : ESP_FT_ERR_OPEN_FAILED;
        free(path);
        return err;
    }
    FILE *opened = fdopen(fd, "wb+");
    if (!opened) {
        close(fd);
        unlink(path);
        free(path);
        return ESP_FT_ERR_OPEN_FAILED;
    }
    *temp_path = path;
    *file = opened;
    return ESP_OK;
}

esp_err_t ft_fs_get_free_bytes(const char *recv_dir, uint64_t *free_bytes)
{
    if (!recv_dir || !free_bytes) {
        return ESP_ERR_INVALID_ARG;
    }
    struct statvfs info;
    if (statvfs(recv_dir, &info) != 0) {
        return errno == ENOSYS ? ESP_ERR_NOT_SUPPORTED : ESP_FAIL;
    }
    if (info.f_frsize != 0 && info.f_bavail > UINT64_MAX / info.f_frsize) {
        *free_bytes = UINT64_MAX;
    } else {
        *free_bytes = (uint64_t)info.f_bavail * info.f_frsize;
    }
    return ESP_OK;
}

static esp_err_t make_candidate(const char *file_name, uint64_t suffix,
                                char candidate[ESP_FT_FILE_NAME_MAX_LEN + 1])
{
    if (suffix == 0) {
        strcpy(candidate, file_name);
        return ESP_OK;
    }
    const char *dot = strrchr(file_name, '.');
    if (dot == file_name) {
        dot = NULL;
    }
    size_t stem_len = dot ? (size_t)(dot - file_name) : strlen(file_name);
    const char *extension = dot ? dot : "";
    size_t extension_len = strlen(extension);
    char suffix_text[24];
    int suffix_len = snprintf(suffix_text, sizeof(suffix_text), "_%" PRIu64, suffix);
    if (suffix_len <= 0 || (size_t)suffix_len + extension_len >= ESP_FT_FILE_NAME_MAX_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }
    size_t max_stem = ESP_FT_FILE_NAME_MAX_LEN - (size_t)suffix_len - extension_len;
    if (stem_len > max_stem) {
        stem_len = max_stem;
        while (stem_len > 0 && (((uint8_t)file_name[stem_len] & 0xc0) == 0x80)) {
            --stem_len;
        }
    }
    memcpy(candidate, file_name, stem_len);
    memcpy(candidate + stem_len, suffix_text, (size_t)suffix_len);
    memcpy(candidate + stem_len + (size_t)suffix_len, extension, extension_len + 1);
    return ESP_OK;
}

static esp_err_t target_available(const char *path, bool *available)
{
    struct stat info;
    if (stat(path, &info) == 0) {
        *available = false;
        return ESP_OK;
    }
    if (errno == ENOENT) {
        *available = true;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t ft_fs_commit_temp(const char *recv_dir, const char *temp_path, const char *file_name,
                            char **target_path, char saved_name[ESP_FT_FILE_NAME_MAX_LEN + 1])
{
    if (!recv_dir || !temp_path || !ft_fs_name_valid(file_name) || !target_path || !saved_name) {
        return ESP_ERR_INVALID_ARG;
    }
    for (uint64_t suffix = 0;; ++suffix) {
        char candidate[ESP_FT_FILE_NAME_MAX_LEN + 1];
        esp_err_t err = make_candidate(file_name, suffix, candidate);
        if (err != ESP_OK) {
            return err;
        }
        char *path = join_path(recv_dir, candidate);
        if (!path) {
            return ESP_ERR_NO_MEM;
        }
        bool available;
        err = target_available(path, &available);
        if (err != ESP_OK) {
            free(path);
            return err;
        }
        if (!available) {
            free(path);
            continue;
        }
        if (rename(temp_path, path) == 0) {
            strcpy(saved_name, candidate);
            *target_path = path;
            return ESP_OK;
        }
        int rename_errno = errno;
        free(path);
        if (rename_errno != EEXIST && rename_errno != ENOTEMPTY) {
            return ESP_FAIL;
        }
        if (suffix == UINT64_MAX) {
            return ESP_ERR_INVALID_SIZE;
        }
    }
}

void ft_fs_remove(const char *path)
{
    if (path && unlink(path) != 0 && errno != ENOENT) {
        ESP_LOGW(TAG, "failed to remove %s: errno=%d", path, errno);
    }
}
