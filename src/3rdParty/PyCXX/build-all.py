#!/usr/bin/python3
import sys
import os
import subprocess

first_limited_api_minor = 4

# (major, minor, bits, vc_ver)
default_versions_to_test = [
    (2,  7, 32,  '9.0'), (2,  7, 64,  '9.0'),
    (3,  3, 32, '10.0'),     # no compiler for windows 64 bit
    (3,  4, 32, '10.0'),     # no compiler for windows 64 bit
    (3,  5, 32, '14.0'), (3,  5, 64, '14.0'),
    (3,  6, 32, '2017'), (3,  6, 64, '2017'),
    (3,  7, 32, '2017'), (3,  7, 64, '2017'),
    (3,  8, 32, '2017'), (3,  8, 64, '2017'),
    (3,  9, 32, '2019'), (3,  9, 64, '2019'),
    (3, 10, 32, '2019'), (3, 10, 64, '2019'),
    (3, 11, 32, '14.0'), (3, 11, 64, '14.0'),
    (3, 12, 32, '14.0'), (3, 12, 64, '14.0'),
]

def main( argv ):
    dry_run = False
    all_versions_to_test = []

    setup_makefile_options = []

    for arg in argv[1:]:
        if arg == '--dry-run':
            dry_run = True

        elif arg.startswith( '--' ):
            # assume its for setup_makefile
            setup_makefile_options.append( arg )

        else:
            # convert all the args into a list of versions to test
            # assume py.exe format maj.min or maj.min-bits
            try:
                if '-' in arg:
                    major_minor, bits = arg.split('-')

                else:
                    major_minor = arg
                    bits = '64'

                major, minor = major_minor.split('.')

                major = int(major)
                minor = int(minor)
                bits = int(bits)

            except ValueError:
                print( 'Error: Expecting <major>.<minor>[-<bits>] 3.9-64 given %r' % (arg,) )
                return 1

            # find the vc_ver from the default_versions_to_test
            vc_ver = None
            for info in default_versions_to_test:
                if info[:3] == (major, minor, bits):
                    vc_ver = info[3]

            if vc_ver is None:
                print( 'Error: Update default_versions_to_test for %d.%d-%d - vc_ver cannot be determined' %
                            (major, minor, bits) )
                return 1

            all_versions_to_test.append( (major, minor, bits, vc_ver) )

    if len(all_versions_to_test) == 0:
        all_versions_to_test = default_versions_to_test

    #
    #   Run all the requested builds
    #
    is_win = sys.platform.startswith( 'win' )

    tag = []
    if os.environ.get('TAG'):
        tag = [os.environ.get('TAG')]

    for major, minor, bits, vc_ver in all_versions_to_test:
        if is_win:
            fmt = '.\\%s.cmd'

        else:
            # Only windows needs to build both 32 and 64 bit
            # for the mac and linux only build once
            if bits == 32:
                continue

            fmt = './%s.sh'

        if is_win:
            cmd = [fmt % ('build-unlimited-api',), '%d.%d' % (major, minor), 'win%d' % (bits,), vc_ver] + setup_makefile_options

        else:
            cmd = [fmt % ('build-unlimited-api',), 'python%d.%d' % (major, minor)] + setup_makefile_options

        print( 'Info: %s' % (' '.join(cmd),), flush=True )
        if not dry_run:
            subprocess.run( cmd )

        if major == 2:
            continue

        for api_minor in range( first_limited_api_minor, minor+1 ):
            if is_win:
                cmd = [fmt % ('build-limited-api',), '%d.%d' % (major, minor), 'win%d' % (bits,), vc_ver]

            else:
                cmd = [fmt % ('build-limited-api',), 'python%d.%d' % (major, minor)]

            # add the API version to use
            cmd.append( '%d.%d' % (major, api_minor) )
            cmd.extend( setup_makefile_options )
            print( 'Info: %s' % (' '.join(cmd),), flush=True )
            if not dry_run:
                subprocess.run( cmd )

    return 0

if __name__ == '__main__':
    sys.exit( main( sys.argv ) )
