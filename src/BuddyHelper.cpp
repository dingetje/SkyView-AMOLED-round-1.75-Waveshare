#include "BuddyHelper.h"
#include <TFT_eSPI.h>
#include <DebugLog.h>

buddy_info_t buddies[MAX_BUDDIES + 1]; // Allocate buddy array

void BuddyManager::readBuddyList(const char* path) 
{
  clearBuddyList(); // 💡 Always clear before loading new

  File file = SPIFFS.open(path, "r");
  if (!file) 
  {
    PRINTLN("Failed to open buddylist.txt for reading");
    return;
  }

  int index = 0;
  while (file.available() && index < MAX_BUDDIES) 
  {
    String line = file.readStringUntil('\n');
    line.trim();

    int commaIndex = line.indexOf(',');
    if (commaIndex != -1) 
    {
      String idStr = line.substring(0, commaIndex);
      String name = line.substring(commaIndex + 1);
      idStr.trim();
      name.trim();

      if (idStr.length() == 0 || name.length() == 0) 
      {
        continue;
      }

      uint32_t buddyId = strtol(idStr.c_str(), NULL, 16);
      buddies[index].id = buddyId;
      buddies[index].name = strdup(name.c_str()); //  Allocate
      index++;
    }
  }
  file.close();

  // Sentinel
  buddies[index].id = 0xFFFFFFFF;
  buddies[index].name = "";
}

void BuddyManager::printBuddyList() 
{
  for (int i = 0; buddies[i].id != 0xFFFFFFFF; i++) 
  {
    Serial.printf("ID: 0x%06X, Name: %s\n", buddies[i].id, buddies[i].name);
  }
}

const char* BuddyManager::findBuddyName(uint32_t id) 
{
    for (int i = 0; buddies[i].id != 0xFFFFFFFF; i++) 
    {
      if (buddies[i].id == id) 
      {
        return buddies[i].name;
      }
    }
    return nullptr;
}
  
String BuddyManager::getBuddyInitials(uint32_t id) 
{
  const char* name = findBuddyName(id);
  if (!name || name[0] == '\0') return "  ";

  String initials = String(name[0]);
  const char* space = strchr(name, ' ');
  if (space && *(space + 1) != '\0') 
  {
    initials += String(*(space + 1));
  }
  return initials;
}

void BuddyManager::clearBuddyList() 
{
  for (int i = 0; i < MAX_BUDDIES; i++) 
  {
    if (buddies[i].name && buddies[i].name[0] != '\0') 
    {
      free((void*)buddies[i].name);  //  Free memory from strdup
      buddies[i].name = nullptr;
    }
    buddies[i].id = 0;
  }

  // Clear sentinel
  buddies[MAX_BUDDIES].id = 0xFFFFFFFF;
  buddies[MAX_BUDDIES].name = "";
}
