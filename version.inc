# Included by CMakeLists, edited by the build script
# (the version numbers are generated by the build script from the git current label)

# name of the slicer
set(SLIC3R_APP_NAME "SliCR-3D")

# Key fo the slicer, must only contains ascii chars and no spaces (be valid in all filesystems)
set(SLIC3R_APP_KEY "SliCR-3D")

# exe name of the slicer, should be lowercase and valid in all os
set(SLIC3R_APP_CMD "SliCR-3D")

# versions
set(SLIC3R_VERSION "2.6.0.3")
set(SLIC3R_VERSION_FULL "2.6.0.3")
set(SLIC3R_BUILD_ID "${SLIC3R_APP_KEY}_${SLIC3R_VERSION_FULL}_2024-07-02")
set(SLIC3R_RC_VERSION "2,6,0,3")
set(SLIC3R_RC_VERSION_DOTS "${SLIC3R_VERSION_FULL}")

# Same as the slicer name but for gcodeviewer
set(GCODEVIEWER_APP_NAME "${SLIC3R_APP_NAME} G-code Viewer")
set(GCODEVIEWER_APP_KEY "${SLIC3R_APP_KEY}-gcodeviewer")
set(GCODEVIEWER_APP_CMD "${SLIC3R_APP_CMD}-gcodeviewer")

# string to be added after the SLIC3R_APP_NAME on some places
set(SLIC3R_BASED_ON " based on PrusaSlicer and SuperSlicer")
# to get he github repo
set(SLIC3R_GITHUB "CR-3D/SliCR-3D")
# download url
set(SLIC3R_DOWNLOAD "https://github.com/CR-3D/SliCR-3D/releases")
set(SLIC3R_DOC_URL "https://manual.slic3r.org/")
