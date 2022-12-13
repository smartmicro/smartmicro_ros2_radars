#!/bin/bash
set -e

smart_pack=SmartAccessAutomotive_2_2_3.tar.gz
URL_smartbinaries=https://www.smartmicro.com/fileadmin/media/Downloads/Automotive_Radar/Software/${smart_pack}

cat << EOF

The following clause is explicit for the Smart Access release.

*********************************************************************************

This software is licensed under the Apache 2.0 License

Copyright (c) 2021, s.m.s, smart microwave sensors GmbH, Brunswick, Germany

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The Software is provided "as is", without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular
purpose and noninfringement. In no event shall the authors or copyright holders be liable for any claim, damages or other liability, whether in an action of contract, tort or
otherwise, arising from, out of or in connection with the software or the use or other dealings in the Software.

*********************************************************************************
EOF
echo
echo -n "Do you accept the agreement you just read? (yes/no)"
echo ""
read REPLY
echo ""
case "$REPLY" in
    yes)
    echo "You have accepted the agreement."
    ;;
    *)
    echo "Agreement not accepted."
    exit
esac
echo

function getSmartaccessBinaries {
    wget -c $URL_smartbinaries
    echo "extracting smart access"
    tar xfz $smart_pack -C umrr_ros2_driver/smartmicro/
}

function cleanup {
    rm -rf $smart_pack
}

getSmartaccessBinaries
cleanup
