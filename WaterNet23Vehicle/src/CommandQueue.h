#define MSG_QUEUE_SIZE 16
#define MSG_MAX_LEN 128
#include "Particle.h"
#include "WaterNet23Vehicle.h"

struct CommandMsg {
    char msg[MSG_MAX_LEN];
    uint8_t mode;
    bool sendAck;
};

class CommandQueue {
public:
    CommandQueue() : head(0), tail(0), count(0) {}
    bool push(const char* str, uint8_t mode, bool sendAck) {
        queueMutex.lock();
        if (count >= MSG_QUEUE_SIZE) {
            queueMutex.unlock();
            return false; // Queue full
        }
        strncpy(buffer[tail].msg, str, MSG_MAX_LEN-1);
        buffer[tail].msg[MSG_MAX_LEN-1] = '\0';
        buffer[tail].mode = mode;
        buffer[tail].sendAck = sendAck;
        tail = (tail + 1) % MSG_QUEUE_SIZE;
        count++;
        queueMutex.unlock();
        return true;
    }
    bool pop(CommandMsg& out) {
        queueMutex.lock();
        if (count == 0) {
            queueMutex.unlock();
            return false;
        }
        out = buffer[head];
        head = (head + 1) % MSG_QUEUE_SIZE;
        count--;
        queueMutex.unlock();
        return true;
    }
    bool isEmpty() {
        queueMutex.lock();
        bool empty = (count == 0);
        queueMutex.unlock();
        return empty;
    }
private:
    CommandMsg buffer[MSG_QUEUE_SIZE];
    int head, tail, count;
    Mutex queueMutex;
};

// Command handler function type
typedef void (*CommandHandler)(const char* dataStr, uint8_t mode);

// Command structure for lookup table
struct CommandEntry {
    const char* cmd;
    CommandHandler handler;
};
