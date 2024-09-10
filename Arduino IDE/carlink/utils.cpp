//#include "utils.h"
//
//void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
//  Serial.printf("Listing directory: %s\n", dirname);
//
//  File root = fs.open(dirname);
//  if (!root) {
//    Serial.println("Failed to open directory");
//    return;
//  }
//  if (!root.isDirectory()) {
//    Serial.println("Not a directory");
//    return;
//  }
//
//  File file = root.openNextFile();
//  while (file) {
//    if (file.isDirectory()) {
//      Serial.print("DIR : ");
//      Serial.println(file.name());
//      if (levels) {
//        listDir(fs, file.name(), levels - 1);
//      }
//    } else {
//      Serial.print("FILE: ");
//      Serial.print(file.name());
//      Serial.print("  SIZE: ");
//      Serial.println(file.size());
//    }
//    file = root.openNextFile();
//  }
//}
//
//// Function to read file content into a char array
//void readFileToBuffer(const char* path, char* buffer, size_t bufferSize) {
//  File file = SPIFFS.open(path, "r");
//  if (file) {
//    size_t len = file.size();
//    if (len > bufferSize - 1) {
//      len = bufferSize - 1;
//    }
//    file.readBytes(buffer, len);
//    buffer[len] = '\0'; // Ensure null-termination
//    file.close();
//  } else {
//    Serial.printf("Failed to open file: %s\n", path);
//  }
//}
