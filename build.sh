if [[ ${HOST} =~ .*darwin.* ]]; then
    # add hacks for osx here!
    echo "adding hacks for osx"

    # # install space-mouse
    # /usr/bin/curl -o /tmp/3dFW.dmg -L 'https://download.3dconnexion.com/drivers/mac/10-7-0_B564CC6A-6E81-42b0-82EC-418EA823B81A/3DxWareMac_v10-7-0_r3411.dmg'
    # hdiutil attach -readonly /tmp/3dFW.dmg
    # sudo installer -package /Volumes/3Dconnexion\ Software/Install\ 3Dconnexion\ software.pkg -target /
    # diskutil eject /Volumes/3Dconnexion\ Software
    CMAKE_PLATFORM_FLAGS+=(-DFREECAD_USE_3DCONNEXION:BOOL=ON)
    CMAKE_PLATFORM_FLAGS+=(-D3DCONNEXIONCLIENT_FRAMEWORK:FILEPATH="/Library/Frameworks/3DconnexionClient.framework")

    CXXFLAGS="${CXXFLAGS} -D_LIBCPP_DISABLE_AVAILABILITY"
fi

cmake \
    --preset conda-macos-release \
    -D CMAKE_IGNORE_PREFIX_PATH="/opt/homebrew;/usr/local/homebrew" \
    -D CMAKE_INSTALL_PREFIX:FILEPATH="$PREFIX" \
    -B build \
    -S . \
    ${CMAKE_PLATFORM_FLAGS[@]}

cmake --build build
cmake --install build

# mv ${PREFIX}/bin/FreeCAD ${PREFIX}/bin/freecad
# mv ${PREFIX}/bin/FreeCADCmd ${PREFIX}/bin/freecadcmd
