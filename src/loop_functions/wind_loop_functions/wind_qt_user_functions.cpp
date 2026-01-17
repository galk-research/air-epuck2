#include "wind_qt_user_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/angles.h>

#include <cmath>   /* for std::sin / std::cos */

using namespace argos;

/* ------------------------------------------------------------------ */
/*  keep pointer to loop-functions                                    */
/* ------------------------------------------------------------------ */
void CWindQTUserFunctions::Init(TConfigurationNode&) {
   m_pcLoop = &dynamic_cast<const CWindLoopFunctions&>(
                 CSimulator::GetInstance().GetLoopFunctions());
}

/* ------------------------------------------------------------------ */
/*  draw wind vector every frame (shaft + arrow-head)                 */
/* ------------------------------------------------------------------ */
void CWindQTUserFunctions::DrawInWorld() {
   const CVector2& cWind = m_pcLoop->GetWind();
   if(cWind.Length() < 0.01) return;

   /* === tweakable parameters ====================================== */
   constexpr Real SCALE        = 0.03;                 /* length factor   */
   constexpr Real HEAD_FRAC    = 0.25;                 /* head/shaft      */
   constexpr Real HEAD_RAD     = ARGOS_PI * 25.0 / 180.0; /* 25° in rad   */
   constexpr Real  RAY_WIDTH   = 5.0;                  /* pixels          */
   /* =============================================================== */

   /* shaft endpoints in 3-D space (arena z ≈ 0) */
   CVector3 from(0, 0, 0.002);
   CVector3 to  (cWind.GetX() * SCALE,
                 cWind.GetY() * SCALE,
                 0.002);

   /* normalized 2-D direction of the wind */
   CVector2 dir = cWind;
   dir.Normalize();

   /* length of head rays */
   Real head_len = cWind.Length() * SCALE * HEAD_FRAC;

   /* rotate ‘dir’ by ±HEAD_RAD to get head directions */
   auto rotate = [](const CVector2& v, Real ang) {
      return CVector2(
         v.GetX()*std::cos(ang) - v.GetY()*std::sin(ang),
         v.GetX()*std::sin(ang) + v.GetY()*std::cos(ang));
   };
   CVector2 dir_left  = rotate(dir,  HEAD_RAD);
   CVector2 dir_right = rotate(dir, -HEAD_RAD);

   /* 3-D endpoints of the two head rays */
   CVector3 head_left ( to.GetX() - dir_left.GetX()*head_len,
                        to.GetY() - dir_left.GetY()*head_len,
                        0.002 );
   CVector3 head_right( to.GetX() - dir_right.GetX()*head_len,
                        to.GetY() - dir_right.GetY()*head_len,
                        0.002 );

   /* --- draw shaft --- */
   DrawRay(CRay3(from, to), CColor::RED, RAY_WIDTH);

   /* --- draw arrow-head (two short rays) --- */
   DrawRay(CRay3(to, head_left ), CColor::RED, RAY_WIDTH);
   DrawRay(CRay3(to, head_right), CColor::RED, RAY_WIDTH);
}

/* ------------------------------------------------------------------ */
/*  register with ARGoS                                               */
/* ------------------------------------------------------------------ */
REGISTER_QTOPENGL_USER_FUNCTIONS(CWindQTUserFunctions,
                                 "wind_qt_user_functions")
