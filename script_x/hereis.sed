#!/bin/bash
echo -n 'waht is the value?'
read value
sed 's/XYZ/'"$value"'/' << EOF
The value is XYZ
EOF

