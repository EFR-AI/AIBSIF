import ast



def engine_writer(eng_pow,filename):

    """
    This function is used to save an engine power list eng to the text file 'filename' in the engines directory
    """

    f=open('engines/' + filename + '.txt','w')
    for ele in eng_pow:
        f.write(str(ele)+'\n')

    f.close()

    return None

def engine_reader(filepath):

    """
    This function is used to read an engine power file in filepath and create a power list from it (That can then be use to create an engine object)
    """

    eng_pow = []
    with open(filepath) as f:
        eng_pow = [ast.literal_eval(line.rstrip()) for line in f]

    return eng_pow

#________________________________________________________#
#                                                        #
#           Use the following for testing                #
#________________________________________________________#



# moteur_20S_astreos = [[0.100, 760.3],
#                     [0.200, 1200.0],
#                     [0.300, 2500.0],
#                     [0.400, 5000.0],
#                     [0.500, 5000.0],
#                     [0.600, 5000.0],
#                     [0.700, 5000.0],
#                     [0.800, 5000.0],
#                     [0.900, 5000.0],
#                     [1.000, 5000.0],
#                     [2.000, 5000.0],
#                     [3.000, 5000.0],
#                     [4.000, 5000.0],
#                     [5.000, 5000.0],
#                     [6.000, 5000.0],
#                     [7.000, 5000.0],
#                     [8.000, 5000.0],
#                     [9.000, 5000.0],
#                     [10.000, 5000.0],
#                     [11.000, 5000.0],
#                     [12.000, 5000.0],
#                     [13.000, 5000.0],
#                     [14.000, 5000.0],
#                     [15.000, 5000.0],
#                     [16.000, 5000.0],
#                     [17.000, 5000.0],
#                     [18.000, 5000.0],
#                     [19.000, 5000.0],
#                     [20.000, 5000.0],
#                     [20.100, 0.0]]

# engine_writer(moteur_20S_astreos, 'test_file') #Will create an engine power file named test file in /engines

# test_list = engine_reader('rockets/engines/thrust_20S_astreos.txt') #Generates an engine power list from an engine power file
# print(test_list)
