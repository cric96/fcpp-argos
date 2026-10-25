// Qt headers first: FCPP defines short macros (CALL, CODE, ...) that must not leak into them.
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "fcpp_argos.hpp"

// Declared by FCPP deployments but left to the platform; never relevant here since uids are always given.
fcpp::device_t fcpp::os::uid() {
    return 0;
}

//! @brief Draws each robot's LED colour as a halo around it (the real LEDs are tiny), plus its label.
class CFcppQTUserFunctions : public argos::CQTOpenGLUserFunctions {
  public:
    CFcppQTUserFunctions() {
        RegisterUserFunction<CFcppQTUserFunctions, argos::CFootBotEntity>(&CFcppQTUserFunctions::Draw);
    }

    void Draw(argos::CFootBotEntity& c_entity) {
        auto& cController = dynamic_cast<CFcppController&>(c_entity.GetControllableEntity().GetController());
        DrawCylinder(argos::CVector3(0, 0, 0), argos::CQuaternion(), 0.14, 0.01, cController.Color);
        DrawText(argos::CVector3(0, 0, 0.35), cController.Label);
    }
};

REGISTER_QTOPENGL_USER_FUNCTIONS(CFcppQTUserFunctions, "fcpp_qtuser_functions")
