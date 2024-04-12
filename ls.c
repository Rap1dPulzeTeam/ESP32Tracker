#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <stdbool.h>

// 结构体用于存储文件和目录的名称以及类型
typedef struct {
    char* name;
    bool is_directory;
} FileInfo;

// 函数声明
int list_directory(const char* path, FileInfo** files);

// 列出目录中的文件和子目录，并使用 malloc 分配内存
int list_directory(const char* path, FileInfo** files) {
    DIR* dir = opendir(path);
    if (!dir) {
        perror("opendir");
        return -1;
    }

    struct dirent* entry;
    int count = 0;
    while ((entry = readdir(dir)) != NULL) {
        // 跳过 "." 和 ".."
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // 构建完整的文件路径
        char* full_path = malloc(strlen(path) + strlen(entry->d_name) + 2); // 加上斜杠和空字符
        if (!full_path) {
            perror("malloc");
            return -1;
        }
        sprintf(full_path, "%s/%s", path, entry->d_name);

        // 判断文件类型
        bool is_directory = entry->d_type == DT_DIR;

        // 将文件/目录信息存储到数组中
        *files = realloc(*files, (count + 1) * sizeof(FileInfo));
        if (!*files) {
            perror("realloc");
            return -1;
        }
        (*files)[count].name = strdup(full_path);
        (*files)[count].is_directory = is_directory;
        count++;

        free(full_path);
    }

    closedir(dir);
    return count;
}

int main() {
    const char* path = "."; // 设置为当前目录
    FileInfo* files = NULL;
    int count = list_directory(path, &files);
    if (count < 0) {
        printf("Failed to list directory.\n");
        return 1;
    }

    printf("Total files and directories: %d\n", count);
    for (int i = 0; i < count; i++) {
        printf("%s - %s\n", files[i].name, files[i].is_directory ? "Directory" : "File");
        free(files[i].name);
    }
    free(files);

    return 0;
}

