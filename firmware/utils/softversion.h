/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file softversion.h
 * @author Gilles Henrard
 */
#ifndef INC_SOFTWARE_VERSION_H
#define INC_SOFTWARE_VERSION_H

enum {
    kFirmwareVersionSize = 7U,  ///< Maximum size of the firmware version string
    kCommitHashSize = 7U,       ///< Maximum size of the Git commit hash code
};

extern const char kGitCommitHash[kCommitHashSize + 1U];
extern const char kFirmwareVersion[kFirmwareVersionSize + 1U];

#endif
