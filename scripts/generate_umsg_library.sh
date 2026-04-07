#Remove old definitions
rm -r umsg/umsg_lib

# Generate umsg_lib based on the HILT message definitions
python3 umsg/umsg/umsg_gen/umsg_gen/umsg_gen.py -d umsg/umsg_def/msg_defs_hitl -o umsg/umsg_lib -l