#pragma once

#include "selfdrive/frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotDataPanel : public QWidget {
  Q_OBJECT

public:
  explicit FrogPilotDataPanel(FrogPilotSettingsWindow *parent);

private:
  void updateState(const UIState &s);

  FrogPilotSettingsWindow *parent;

  Params params;

  bool utilityActive;
};
