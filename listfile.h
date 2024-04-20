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
        if (strcmp(entry->d_name, ".") == 0) {// || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // 构建完整的文件路径
        char* full_path = (char*)malloc(strlen(entry->d_name) + 2); // 加上斜杠和空字符
        if (!full_path) {
            perror("malloc");
            return -1;
        }
        sprintf(full_path, "%s", entry->d_name);

        // 判断文件类型
        bool is_directory = entry->d_type == DT_DIR;

        // 将文件/目录信息存储到数组中
        *files = (FileInfo*)realloc(*files, (count + 1) * sizeof(FileInfo));
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

char* shortenFileName(const char *fileName, uint8_t maxLength) {
    uint8_t len = strlen(fileName);
    uint8_t extensionLength = 6; // 扩展名长度为4

    // 如果文件名长度已经小于等于最大长度，不需要截取
    if (len <= maxLength) {
        return strdup(fileName);
    }

    // 计算需要保留的文件名部分的长度（包括省略号和扩展名）
    int8_t remainingLength = maxLength - extensionLength - 2; // 留出空间给省略号和空格

    // 分配内存来存储截取后的文件名
    char *shortenedName = (char *)malloc((maxLength + 1) * sizeof(char));
    snprintf(shortenedName, maxLength + 1, "%.*s..%s", remainingLength, fileName, fileName + len - extensionLength);

    return shortenedName;
}