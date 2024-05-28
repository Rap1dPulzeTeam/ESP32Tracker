#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ssd1306.h"
#include "freertos/task.h"

#define WIDTH 64
#define HEIGHT 16

typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    Point *body;
    int length;
    int capacity;
} Snake;

typedef struct {
    Point pos;
    int cost;
    int priority;
} AStarNode;

typedef struct {
    AStarNode *nodes;
    int size;
    int capacity;
} PriorityQueue;

void init_snake(Snake *snake) {
    snake->capacity = 10000;
    snake->length = 3;
    snake->body = (Point *)malloc(snake->capacity * sizeof(Point));
    snake->body[0] = (Point){WIDTH / 4, HEIGHT / 2};
    snake->body[1] = (Point){WIDTH / 4 - 1, HEIGHT / 2};
    snake->body[2] = (Point){WIDTH / 4 - 2, HEIGHT / 2};
}

void init_food(Point *food, Snake *snake) {
    while (1) {
        food->x = rand() % WIDTH;
        food->y = rand() % HEIGHT;
        bool on_snake = false;
        for (int i = 0; i < snake->length; i++) {
            if (snake->body[i].x == food->x && snake->body[i].y == food->y) {
                on_snake = true;
                break;
            }
        }
        if (!on_snake) break;
    }
}

bool is_valid(Point p, Snake *snake) {
    if (p.x < 0 || p.x >= WIDTH || p.y < 0 || p.y >= HEIGHT) return false;
    for (int i = 0; i < snake->length; i++) {
        if (snake->body[i].x == p.x && snake->body[i].y == p.y) return false;
    }
    return true;
}

void init_priority_queue(PriorityQueue *pq, int capacity) {
    pq->capacity = capacity;
    pq->size = 0;
    pq->nodes = (AStarNode *)malloc(pq->capacity * sizeof(AStarNode));
}

void push(PriorityQueue *pq, AStarNode node) {
    if (pq->size >= pq->capacity) return;
    pq->nodes[pq->size++] = node;
}

AStarNode pop(PriorityQueue *pq) {
    int best_idx = 0;
    for (int i = 1; i < pq->size; i++) {
        if (pq->nodes[i].priority < pq->nodes[best_idx].priority) {
            best_idx = i;
        }
    }
    AStarNode best_node = pq->nodes[best_idx];
    pq->nodes[best_idx] = pq->nodes[--pq->size];
    return best_node;
}

bool is_empty(PriorityQueue *pq) {
    return pq->size == 0;
}

int heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

bool astar(Snake *snake, Point food, Point *direction) {
    Point directions[] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    PriorityQueue queue;
    init_priority_queue(&queue, WIDTH * HEIGHT);
    push(&queue, (AStarNode){snake->body[0], 0, heuristic(snake->body[0], food)});
    
    Point **came_from = (Point **)malloc(HEIGHT * sizeof(Point *));
    for (int i = 0; i < HEIGHT; i++) {
        came_from[i] = (Point *)malloc(WIDTH * sizeof(Point));
    }
    
    int **cost_so_far = (int **)malloc(HEIGHT * sizeof(int *));
    for (int i = 0; i < HEIGHT; i++) {
        cost_so_far[i] = (int *)malloc(WIDTH * sizeof(int));
        for (int j = 0; j < WIDTH; j++) {
            cost_so_far[i][j] = -1;
        }
    }
    
    came_from[snake->body[0].y][snake->body[0].x] = (Point){-2, -2};
    cost_so_far[snake->body[0].y][snake->body[0].x] = 0;

    while (!is_empty(&queue)) {
        AStarNode current = pop(&queue);
        if (current.pos.x == food.x && current.pos.y == food.y) {
            Point next = current.pos;
            while (came_from[next.y][next.x].x != -2) {
                Point prev = came_from[next.y][next.x];
                if (came_from[prev.y][prev.x].x == -2) {
                    *direction = (Point){next.x - prev.x, next.y - prev.y};
                    for (int i = 0; i < HEIGHT; i++) {
                        free(came_from[i]);
                        free(cost_so_far[i]);
                    }
                    free(came_from);
                    free(cost_so_far);
                    free(queue.nodes);
                    return true;
                }
                next = prev;
            }
        }

        for (int i = 0; i < 4; i++) {
            Point next = {current.pos.x + directions[i].x, current.pos.y + directions[i].y};
            if (is_valid(next, snake)) {
                int new_cost = cost_so_far[current.pos.y][current.pos.x] + 1;
                if (cost_so_far[next.y][next.x] == -1 || new_cost < cost_so_far[next.y][next.x]) {
                    cost_so_far[next.y][next.x] = new_cost;
                    int priority = new_cost + heuristic(next, food);
                    push(&queue, (AStarNode){next, new_cost, priority});
                    came_from[next.y][next.x] = current.pos;
                }
            }
        }
    }
    for (int i = 0; i < HEIGHT; i++) {
        free(came_from[i]);
        free(cost_so_far[i]);
    }
    free(came_from);
    free(cost_so_far);
    free(queue.nodes);
    return false;
}

void move_snake(Snake *snake, Point direction) {
    for (int i = snake->length - 1; i > 0; i--) {
        snake->body[i] = snake->body[i - 1];
    }
    snake->body[0].x += direction.x;
    snake->body[0].y += direction.y;
}

void grow_snake(Snake *snake) {
    if (snake->length < snake->capacity) {
        snake->body[snake->length] = snake->body[snake->length - 1];
        snake->length++;
    }
}

void draw_game(Snake *snake, Point food, SSD1306_t *dev) {
    ssd1306_clear_buffer(dev);
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            bool printed = false;
            if (x == food.x && y == food.y) {
                _ssd1306_pixel(dev, x, y, false);
                printed = true;
            } else {
                for (int i = 0; i < snake->length; i++) {
                    if (snake->body[i].x == x && snake->body[i].y == y) {
                        _ssd1306_pixel(dev, x, y, false);
                        printed = true;
                        break;
                    }
                }
            }
            // if (!printed) printf(".");
        }
    }
    ssd1306_show_buffer(dev);
}

void game_over(SSD1306_t *dev) {
    ssd1306_clear_buffer(dev);
    ssd1306_show_buffer(dev);
    ssd1306_display_text(dev, 1, "GAME OVER", 10, false);
    vTaskDelay(2048);
}