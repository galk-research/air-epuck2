#ifndef WIND_QT_USER_FUNCTIONS_H
#define WIND_QT_USER_FUNCTIONS_H

#include "wind_loop_functions.h"
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

/* Pure drawing class (requires Qt-OpenGL) */
class CWindQTUserFunctions final : public argos::CQTOpenGLUserFunctions {

public:
    void Init(argos::TConfigurationNode& t_node) override;
    void DrawInWorld() override;

private:
    const CWindLoopFunctions* m_pcLoop = nullptr;   /* pointer to logic class */
};

#endif /* WIND_QT_USER_FUNCTIONS_H */
