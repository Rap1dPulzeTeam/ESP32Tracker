#ifndef SERIAL_TERMINAL_H
#define SERIAL_TERMINAL_H

#include <Arduino.h>

class SerialTerminal {
public:
    static const int CMD_BUF_SIZE = 64;
    static const int CMD_LIST_SIZE = 10;

    typedef struct {
        const char* name;
        void (*func)(int argc, const char* argv[]);
    } Command;

    SerialTerminal() 
        : cmdBufferIndex(0), cmdListSize(0) {}

    void begin(long baudRate) {
        Serial.begin(baudRate);
        Serial.println("Serial Terminal Started");
        printPrompt();

        // 添加内置的帮助命令
        addCommand("help", helpCmd);
    }

    void update() {
        // 读取串口输入
        if (Serial.available() > 0) {
            char c = Serial.read();

            if (c == 127) { // 处理退格键
                if (cmdBufferIndex > 0) {
                    cmdBufferIndex--;
                    Serial.print("\b \b"); // 光标左移一位并覆盖字符
                }
            } else if (c == '\n' || c == '\r') { // 处理换行符
                if (cmdBufferIndex > 0) {
                    cmdBuffer[cmdBufferIndex] = '\0';
                    Serial.println(); // 回显换行
                    executeCommand(cmdBuffer);
                    cmdBufferIndex = 0;
                } else {
                    Serial.println(); // 直接回车时换行
                }
                printPrompt(); // 打印提示行
            } else if (cmdBufferIndex < CMD_BUF_SIZE - 1) {
                // 记录命令字符并回显
                cmdBuffer[cmdBufferIndex++] = c;
                Serial.print(c);
            }
        }
    }

    void addCommand(const char* name, void (*func)(int argc, const char* argv[])) {
        if (cmdListSize < CMD_LIST_SIZE) {
            cmdList[cmdListSize].name = name;
            cmdList[cmdListSize].func = func;
            cmdListSize++;
        }
    }

    static void helpCmd(int argc, const char* argv[]) {
        Serial.println("Available commands:");
        for (int i = 0; i < instance->cmdListSize; i++) {
            Serial.println(instance->cmdList[i].name);
        }
    }

    void executeCommand(const char* cmd) {
        // 分割命令和参数
        const char* argv[CMD_BUF_SIZE / 2];
        int argc = 0;
        char cmdCopy[CMD_BUF_SIZE];
        strncpy(cmdCopy, cmd, CMD_BUF_SIZE);
        char* token = strtok(cmdCopy, " ");
        while (token != NULL && argc < CMD_BUF_SIZE / 2) {
            argv[argc++] = token;
            token = strtok(NULL, " ");
        }

        if (argc > 0) {
            for (int i = 0; i < cmdListSize; i++) {
                if (strcmp(argv[0], cmdList[i].name) == 0) {
                    cmdList[i].func(argc, argv);
                    return;
                }
            }
            Serial.println("Unknown command");
        }
    }

    static SerialTerminal* instance;

private:
    void printPrompt() {
        Serial.print("ESP32Terminal>>> ");
    }

    char cmdBuffer[CMD_BUF_SIZE];
    int cmdBufferIndex;
    Command cmdList[CMD_LIST_SIZE];
    int cmdListSize;
};

// 定义静态成员变量
SerialTerminal* SerialTerminal::instance = nullptr;

#endif
