#pragma once

#include "VHXPCommon.h"
#include <RDMTypes.h>

class VHXPData
{
  public:
    void Init();
    void ReadData();
    void ComputeLegPositions();

    const RDMVec& GetLegPosition(int i) { return m_legPosition[i]; }

  private:
    float m_servoAngle[18];
    RDMVec m_legPosition[6];
};
