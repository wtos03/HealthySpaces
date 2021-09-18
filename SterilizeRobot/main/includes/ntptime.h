#include <time.h>
#include <sys/time.h>

void setTimeFromNTP(void);
void getTimeStructure (struct tm *);
void diffTimePeriod( struct tm ,struct tm ,struct tm* ); 
