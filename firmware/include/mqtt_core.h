#pragma once

void mqtt_app_start(const char *broker_uri);
void mqtt_discovery_task(void *pvParameters);