#ifndef PUMPKIN_COMMAND_DISPATCH_H
#define PUMPKIN_COMMAND_DISPATCH_H

void CommandDispatch_Execute(const char *line);
void CommandDispatch_ReportStatus(void);
void CommandDispatch_Poll(void);

#endif
