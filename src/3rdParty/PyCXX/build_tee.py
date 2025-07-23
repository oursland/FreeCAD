#!/usr/bin/env python3
import sys
import re

def main( argv ):
    all_copies = []
    i_args = iter( argv )
    next( i_args )

    mode = 'w'

    for filename in i_args:
        if filename == '-a':
            mode = 'a'
        else:
            all_copies.append( open( filename, 'w' ) )

    colour = re.compile( r'\033\[[\d;]*m' )

    for line in sys.stdin:
        # allow colours to be shown seen
        sys.stdout.write( line )

        # remove colouring from log files.
        line = colour.sub( '', line )

        for copy in all_copies:
            copy.write( line )

    return 0

if __name__ == '__main__':
    sys.exit( main( sys.argv ) )
