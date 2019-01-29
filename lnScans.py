import os
import sys
argv = sys.argv
working_directory = os.path.dirname(os.path.realpath(__file__))

if len(argv) < 3: 
    print("Usage: python "+ argv[0] + " insitefolder path_all_scans")
    exit(1)
path_base = os.path.join(working_directory,argv[1])
path_all_scans = os.path.join(working_directory,argv[2])
i = 0
while True:
    number = 'run%05d' % (i)
    path_run = os.path.join(path_base, number)
    path_final = os.path.join(path_all_scans , 'scans_' + number + '.zip')
    if not os.path.exists(path_run):
        print("The "+ path_run + " does not exists. Stopping..")
        break

    os.system('ln -s '+ path_final + ' ' + path_run)
    print('ln -s '+ path_final + ' ' + path_run)
    i += 1
