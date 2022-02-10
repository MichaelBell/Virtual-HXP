#include "VHXPData.h"
#include "VHXPServoInput.h"

static float sCenteredAngle[18] = { 1.602f, 1.453f, 1.681f, 1.429f, 1.343f, 1.728f, 1.319f, 1.367f, 1.728f, 1.210f, 1.319f, 1.581f, 1.257f, 1.312f, 1.602f, 1.335f, 1.367f, 1.539f};

void VHXPData::Init()
{
  servo_input_start();
}

void VHXPData::ReadData()
{
    const float fSERVO_FREQ = 50;
    const float pulseDuration = 1000.0f / (fSERVO_FREQ * 4096);
    for (int i = 0; i < 9; ++i)
    {
        uint16_t on = servo_get_on_duration(0, i);
        uint16_t off = servo_get_off_duration(0, i);
        float pulseInMilliseconds = off * pulseDuration;
        float angle = (pulseInMilliseconds - 1.5f) * HALF_PI + sCenteredAngle[i];
        m_servoAngle[i] = angle;
    }

    for (int i = 1; i < 10; ++i)
    {
        uint16_t on = servo_get_on_duration(1, i);
        uint16_t off = servo_get_off_duration(1, i);
        float pulseInMilliseconds = off * pulseDuration;
        float angle = (pulseInMilliseconds - 1.5f) * HALF_PI + sCenteredAngle[18-i];
        m_servoAngle[18-i] = angle;
    }
}

void VHXPData::ComputeLegPositions()
{
  const float L[3] = { 137.29f, 93.1f, 45.1f };

  for (int i = 0; i < 6; ++i)
  {
    float* angle = &m_servoAngle[i*3];
    RDMVec& leg = m_legPosition[i];
    float alpha = PI - angle[1] - angle[0];
    leg.z = cosf(angle[1]) * L[1] - cosf(alpha) * L[0];
    float u = sinf(angle[1]) * L[1] + sinf(alpha) * L[0] + L[2];
    leg.x = sinf(angle[2]) * u;
    leg.y = cosf(angle[2]) * u;
  }
}
