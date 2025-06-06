#ifndef BUDDY_HELPER_H
#define BUDDY_HELPER_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>

#define MAX_BUDDIES 20

typedef struct {
  uint32_t id;
  const char* name;
} buddy_info_t;

extern buddy_info_t buddies[MAX_BUDDIES + 1]; // 20 buddies + 1 sentinel

class BuddyManager {
public:
  static void readBuddyList(const char* path = "/buddylist.txt");
  static void printBuddyList();
  static const char* findBuddyName(uint32_t id);
  static String getBuddyInitials(uint32_t id);
  static void clearBuddyList();  // 💡 New
};

#endif // BUDDY_HELPER_H
