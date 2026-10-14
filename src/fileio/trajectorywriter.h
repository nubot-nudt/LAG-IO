#pragma once
#include <stdexcept>

#include "common/types.h"
#include "filesaver.h"

inline void writeNavResult(double time, std::shared_ptr<::NavState> navstate, FileSaver& navfile,
                           FileSaver& imuerrfile) {
  if (!std::isfinite(time) || !navstate->pva.pos.allFinite() || !navstate->pva.vel.allFinite() ||
      !navstate->pva.att.euler.allFinite() || !navstate->imuerror.gyrbias.allFinite() ||
      !navstate->imuerror.accbias.allFinite())
    throw std::runtime_error("Cannot export a non-finite estimated state");
  // time, position [m], velocity [m/s], roll/pitch/yaw [deg].
  std::vector<double> result;

  result.clear();
  result.push_back(time);
  result.push_back(navstate->pva.pos[0]);
  result.push_back(navstate->pva.pos[1]);
  result.push_back(navstate->pva.pos[2]);
  result.push_back(navstate->pva.vel[0]);
  result.push_back(navstate->pva.vel[1]);
  result.push_back(navstate->pva.vel[2]);
  result.push_back(navstate->pva.att.euler[0] * R2D);
  result.push_back(navstate->pva.att.euler[1] * R2D);
  result.push_back(navstate->pva.att.euler[2] * R2D);
  navfile.dump(result);

  auto imuerr = navstate->imuerror;
  result.clear();
  result.push_back(time);
  result.push_back(imuerr.gyrbias[0] * R2D * 3600);
  result.push_back(imuerr.gyrbias[1] * R2D * 3600);
  result.push_back(imuerr.gyrbias[2] * R2D * 3600);
  result.push_back(imuerr.accbias[0] * 1e5);
  result.push_back(imuerr.accbias[1] * 1e5);
  result.push_back(imuerr.accbias[2] * 1e5);

  imuerrfile.dump(result);
}
