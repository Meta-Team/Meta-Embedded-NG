#ifndef SENTRY_H
#define SENTRY_H

void SentryInit();

void SentryControlTask();

void SentrySensorTask();

void SentryCommTask();

void SentryWDTTask();

#endif // !SENTRY_H