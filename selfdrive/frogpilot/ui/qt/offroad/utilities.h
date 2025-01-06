#pragma once

#include "selfdrive/frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotUtilitiesPanel : public QWidget {
  Q_OBJECT

public:
  explicit FrogPilotUtilitiesPanel(FrogPilotSettingsWindow *parent);

private:
  void updateState(const UIState &s);

  FrogPilotButtonsControl *forceStartedBtn;

  FrogPilotSettingsWindow *parent;

  Params params;
  Params params_memory{"/dev/shm/params"};

  bool utilityActive;
};
