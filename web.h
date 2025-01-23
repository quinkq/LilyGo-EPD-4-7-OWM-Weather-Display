#pragma once

#ifndef WEB_H
#define WEB_H

void setupWEB(void);
void setupAP(void);
void handleRoot();
void handleSettings();
void handleConfig();
void handleNotFound();
void storeConfig();
void webTask(void *args);

#endif