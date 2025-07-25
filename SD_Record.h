#ifndef SD_RECORD_H
#define SD_RECORD_H

void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);

#endif