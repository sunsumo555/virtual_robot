from math import cos, sin, acos, sqrt, exp, pi
import numpy as np
import montecarlo as mc

positions = [(84,30),
             (180,30),
             (180,54),
             (138,54),
             (138,168)]

ground_truth_measurements = [[84.0, 84.11527706382536, 84.46269548333538, 85.04707056619225, 85.87660996866245, 86.96319915444698, 88.32282683601446, 83.7128432887601, 73.7578000672271, 66.08067793755798, 60.00000000000001, 55.08235376329992, 51.03904850112239, 47.67047187197249, 44.83429649593827, 42.42640687119285, 40.368981888191286, 38.602786976795024, 37.08203932499369, 35.77089878507842, 34.64101615137755, 33.66978712903082, 32.8390883551814, 32.13434980911087, 31.543866727148014, 31.058285412302492, 30.670217845950877, 30.37395377364009, 30.165248386905496, 30.04117037993763, 30.0, 30.04117037993763, 30.165248386905496, 30.37395377364009, 30.670217845950877, 31.058285412302492, 31.543866727148018, 32.13434980911087, 32.8390883551814, 33.66978712903082, 34.64101615137754, 35.77089878507842, 37.08203932499369, 38.60278697679502, 40.36898188819128, 42.42640687119285, 44.834296495938254, 47.67047187197247, 51.039048501122394, 55.082353763299885, 59.99999999999998, 66.08067793755801, 73.75780006722712, 83.71284328876001, 97.0820393249937, 115.91109915468813, 128.81491495299372, 127.57060584928837, 126.69404322500307, 126.17291559573805, 126.0, 126.17291559573805, 126.69404322500306, 127.57060584928837, 128.8149149529937, 130.44479873167046, 132.48424025402167, 134.96426919826564, 132.76404012100895, 118.94522028760441, 108.00000000000004, 100.15851659821956, 103.82971010998233, 108.08780353502604, 113.03314928693555, 118.79393923933998, 125.5360301886271, 133.4773212415229, 142.90933580314268, 154.23059053723955, 167.9999999999999, 185.0258982251624, 197.03453013108847, 192.80609885466524, 189.2632003628881, 186.349712473815, 184.0213070757053, 182.24372264184052, 180.99149032143296, 180.24702227962578, 138.0, 138.18938374771307, 138.76014257976524, 139.7201873587444, 141.083002091374, 142.86811289659144, 145.10178694488087, 147.81800912190997, 151.0598064338344, 154.88102079354178, 159.34867429633664, 154.2305905372398, 142.90933580314274, 133.47732124152304, 125.53603018862718, 118.79393923933999, 113.03314928693567, 108.08780353502608, 103.82971010998233, 100.15851659821963, 96.99484522385714, 94.27540396128632, 91.94944739450796, 89.97617946551044, 88.32282683601446, 86.963199154447, 85.87660996866246, 85.04707056619225, 84.46269548333538, 84.11527706382536],
                             [180.0, 180.24702227962578, 180.99149032143296, 182.24372264184052, 144.29203034232393, 115.91109915468809, 97.08203932499367, 83.7128432887601, 73.7578000672271, 66.08067793755798, 60.00000000000001, 55.08235376329992, 51.03904850112239, 47.67047187197249, 44.83429649593827, 42.42640687119285, 40.368981888191286, 38.602786976795024, 37.08203932499369, 35.77089878507842, 34.64101615137755, 33.66978712903082, 32.8390883551814, 32.13434980911087, 31.543866727148014, 31.058285412302492, 30.670217845950877, 30.37395377364009, 30.165248386905496, 30.04117037993763, 30.0, 30.04117037993763, 30.165248386905496, 30.37395377364009, 30.670217845950877, 31.058285412302492, 31.543866727148018, 32.13434980911087, 32.8390883551814, 33.66978712903082, 34.64101615137754, 35.77089878507842, 37.08203932499369, 38.60278697679502, 40.36898188819128, 42.426406871192846, 40.36898188819129, 38.60278697679503, 37.082039324993694, 35.77089878507842, 34.64101615137755, 33.66978712903082, 32.8390883551814, 32.13434980911087, 31.54386672714802, 31.058285412302496, 30.670217845950884, 30.37395377364009, 30.165248386905493, 30.04117037993763, 30.0, 30.04117037993763, 30.16524838690549, 30.37395377364009, 30.670217845950877, 31.05828541230249, 31.54386672714802, 32.134349809110866, 32.8390883551814, 33.66978712903082, 34.64101615137754, 35.770898785078415, 37.082039324993694, 38.60278697679501, 40.36898188819127, 42.426406871192846, 44.83429649593825, 47.67047187197246, 51.0390485011224, 55.082353763299835, 59.99999999999996, 60.605616832255485, 59.110359039326546, 57.84182965639957, 56.778960108866436, 55.9049137421445, 55.206392122711584, 54.67311679255216, 54.297447096429885, 54.074106683887734, 54.0, 54.07410668388773, 54.29744709642988, 54.67311679255216, 55.20639212271157, 186.34971247381495, 189.26320036288809, 192.80609885466515, 197.0345301310884, 202.0187227741849, 192.00000000000023, 176.26353204255977, 163.32495520359168, 152.54550999031207, 143.4697487870025, 135.76450198781714, 206.23776388131586, 219.28417061107334, 222.49223594996215, 214.62539271047063, 207.8460969082653, 202.01872277418497, 197.03453013108847, 192.80609885466524, 189.2632003628881, 186.349712473815, 184.0213070757053, 182.24372264184052, 180.99149032143296, 180.24702227962578],
                             [180.0, 180.24702227962578, 180.99149032143296, 182.24372264184052, 184.02130707570527, 186.34971247381498, 174.7476707849886, 150.68311791976816, 132.76404012100878, 118.94522028760437, 108.00000000000001, 99.14823677393986, 91.87028730202029, 85.80684936955048, 80.70173369268889, 76.36753236814712, 72.66416739874431, 69.48501655823104, 66.74767078498864, 64.38761781314116, 62.35382907247959, 60.605616832255485, 59.110359039326525, 57.84182965639957, 56.77896010886643, 55.90491374214449, 55.20639212271158, 54.67311679255216, 54.29744709642989, 54.074106683887734, 54.0, 54.074106683887734, 54.29744709642989, 54.67311679255216, 55.20639212271158, 55.90491374214449, 56.778960108866436, 57.84182965639957, 59.110359039326525, 60.605616832255485, 60.00000000000001, 55.08235376329992, 51.0390485011224, 47.67047187197249, 44.834296495938275, 42.426406871192846, 40.36898188819129, 38.60278697679503, 37.082039324993694, 35.77089878507842, 34.64101615137755, 33.66978712903082, 32.8390883551814, 32.13434980911087, 31.54386672714802, 31.058285412302496, 30.670217845950884, 30.37395377364009, 30.165248386905493, 30.04117037993763, 30.0, 30.04117037993763, 30.16524838690549, 30.37395377364009, 30.670217845950877, 31.05828541230249, 31.54386672714802, 32.134349809110866, 32.8390883551814, 33.66978712903082, 34.64101615137754, 35.770898785078415, 37.082039324993694, 38.60278697679501, 40.36898188819127, 42.426406871192846, 40.36898188819129, 38.60278697679503, 37.082039324993694, 35.77089878507844, 34.64101615137755, 33.66978712903082, 32.83908835518142, 32.13434980911087, 31.54386672714802, 31.0582854123025, 30.670217845950884, 30.37395377364009, 30.165248386905493, 30.04117037993763, 30.0, 30.041170379937626, 30.16524838690549, 30.37395377364009, 30.670217845950873, 31.05828541230249, 31.543866727148014, 32.13434980911086, 170.76325944694327, 175.08289307096027, 180.13328398716317, 176.26353204255977, 163.32495520359168, 152.54550999031207, 143.4697487870025, 135.76450198781714, 129.1807420422122, 123.5289183257441, 193.94838430426506, 209.31294430053939, 207.8460969082653, 202.01872277418497, 197.03453013108847, 192.80609885466524, 189.2632003628881, 186.349712473815, 184.0213070757053, 182.24372264184052, 180.99149032143296, 180.24702227962578],
                             [138.0, 138.1893837477131, 138.76014257976527, 139.7201873587444, 141.08300209137403, 142.86811289659147, 145.1017869448809, 147.81800912191002, 132.76404012100878, 118.94522028760437, 108.00000000000001, 99.14823677393986, 91.87028730202029, 85.80684936955048, 80.70173369268889, 76.36753236814712, 72.66416739874431, 69.48501655823104, 66.74767078498864, 64.38761781314116, 62.35382907247959, 60.605616832255485, 59.110359039326525, 57.84182965639957, 56.77896010886643, 55.90491374214449, 55.20639212271158, 54.67311679255216, 54.29744709642989, 54.074106683887734, 54.0, 54.074106683887734, 54.29744709642989, 54.67311679255216, 55.20639212271158, 55.90491374214449, 56.778960108866436, 57.84182965639957, 59.110359039326525, 60.605616832255485, 62.353829072479584, 64.38761781314116, 66.74767078498864, 69.48501655823104, 72.6641673987443, 76.36753236814712, 80.70173369268885, 85.80684936955045, 88.99689437998487, 85.85015708418823, 83.13843876330613, 80.80748910967397, 78.81381205243537, 77.1224395418661, 75.70528014515524, 74.53988498952599, 73.60852283028211, 72.89748905673622, 72.39659612857318, 72.09880891185031, 72.0, 72.09880891185031, 72.39659612857317, 72.89748905673622, 73.6085228302821, 74.53988498952597, 75.70528014515524, 77.12243954186609, 73.7578000672272, 66.08067793755801, 60.00000000000002, 55.08235376329992, 51.0390485011224, 47.670471871972495, 44.834296495938275, 42.426406871192846, 44.83429649593825, 47.67047187197246, 51.03904850112239, 55.08235376329984, 59.99999999999996, 66.080677937558, 73.75780006722702, 83.71284328876, 97.08203932499367, 115.91109915468789, 144.2920303423237, 157.94455962292847, 156.85929161190856, 156.2140859756757, 156.0, 156.21408597567566, 156.85929161190853, 157.94455962292847, 159.48513279894453, 161.50308414397296, 164.02810698116969, 150.6831179197684, 132.76404012100897, 118.94522028760441, 108.00000000000013, 99.14823677393987, 91.87028730202033, 146.69059051182097, 153.40213117512684, 161.22034611053283, 170.37032668456527, 177.57282009325712, 170.577380894971, 164.5461344113608, 159.34867429633672, 154.8810207935418, 151.0598064338345, 147.81800912191002, 145.1017869448809, 142.8681128965915, 141.08300209137406, 139.7201873587444, 138.76014257976527, 138.1893837477131],
                             [54.0, 54.074106683887734, 54.297447096429885, 54.67311679255216, 55.20639212271158, 55.904913742144494, 56.778960108866436, 57.84182965639957, 59.11035903932653, 60.605616832255485, 62.353829072479584, 64.38761781314115, 66.74767078498864, 177.5728200932571, 185.69731668567988, 195.16147160748713, 206.23776388131597, 216.17560707005214, 207.65942021996466, 200.31703319643918, 193.98969044771428, 188.55080792257263, 183.89889478901586, 179.95235893102088, 176.64565367202889, 173.92639830889397, 171.7532199373249, 170.0941411323845, 168.92539096667076, 168.23055412765072, 168.0, 168.23055412765072, 168.92539096667076, 170.0941411323845, 171.7532199373249, 173.92639830889397, 176.6456536720289, 83.71284328876011, 73.75780006722714, 66.080677937558, 60.000000000000014, 55.08235376329991, 51.03904850112239, 47.67047187197249, 44.83429649593827, 42.426406871192846, 40.368981888191286, 38.60278697679503, 37.082039324993694, 35.77089878507842, 34.641016151377556, 33.66978712903082, 32.8390883551814, 32.13434980911087, 31.543866727148018, 31.058285412302492, 30.67021784595088, 30.373953773640086, 30.165248386905493, 30.04117037993763, 30.0, 30.04117037993763, 30.16524838690549, 30.373953773640086, 30.670217845950877, 31.05828541230249, 31.543866727148018, 32.134349809110866, 32.839088355181396, 33.66978712903082, 34.64101615137754, 35.770898785078415, 37.082039324993694, 38.60278697679502, 40.36898188819127, 42.426406871192846, 44.83429649593825, 47.67047187197246, 51.03904850112239, 50.07925829910982, 48.49742261192857, 47.13770198064316, 45.97472369725398, 44.98808973275522, 44.16141341800723, 43.4815995772235, 42.93830498433123, 42.52353528309612, 42.23134774166769, 42.05763853191268, 42.0, 42.05763853191267, 42.23134774166768, 42.52353528309612, 42.93830498433122, 43.481599577223484, 44.16141341800722, 44.988089732755206, 45.97472369725396, 47.13770198064315, 48.49742261192854, 50.079258299109775, 51.914855054991165, 54.04390176751299, 56.51657464346778, 59.39696961966999, 62.76801509431352, 66.73866062076145, 66.74767078498864, 64.38761781314119, 62.35382907247959, 60.605616832255485, 59.110359039326546, 57.84182965639957, 56.778960108866436, 55.9049137421445, 55.206392122711584, 54.67311679255216, 54.297447096429885, 54.074106683887734]]

ground_truth_histograms = [[ 0,  0,  0, 27,  8,  5,  3,  2, 16,  4,  7,  6, 11, 10,  6,  5,  1,
        0,  7,  2,  0,  0,  0,  0,  0],
                           [ 0,  0, 30, 31,  9, 15,  2,  1,  1,  0,  1,  0,  1,  2,  1,  1,  9,
        7,  4,  3,  2,  0,  0,  0],
                           [ 0,  0, 30, 23,  6, 20,  3,  3,  1,  2,  1,  2,  2,  2,  1,  2, 13,
        5,  2,  2,  0,  0,  0,  0],
                           [ 0,  0,  0,  1,  9, 24, 23, 13,  4,  4,  3,  0,  9, 12, 13,  4,  1,
        0,  0,  0,  0,  0,  0,  0],
                           [ 0,  0, 15, 25, 25, 25,  5,  1,  0,  0,  0,  0,  0,  0,  0,  9,  7,
        3,  2,  3,  0,  0,  0,  0]]

def get_ground_truth(landmark_index,points = 120):
    measurements = []
    x,y = positions[landmark_index]
    thetas = np.linspace(-1*pi,pi*(1.0-2.0/points),points)
    for theta in thetas:
        measurements.append(mc.find_actual_distance(x,y,theta))
    return measurements

def get_ground_truth_histograms(landmark_index, points = 120):
    ground_truth = get_ground_truth(landmark_index,points)
    histogram, bins = np.np.histogram(ground_truth,bins = np.linspace(0,250,26))
    return histogram

def find_sum_of_squares_error(a,b):
    return np.sum((np.array(histogram[:-2])-np.array(measured_histogram[:-2]))**2)

def recognize_place(measured_histogram):
    min_error = 9999
    min_index = 0
    for i,histogram in enumerate(ground_truth_histograms):
        error = find_sum_of_squares_error(np.array(histogram[:-2]),np.array(measured_histogram[:-2]))
        if error < min_error:
            min_error = error
            min_index = i
    return min_index

def find_orientation(measured,index):
    min_error = 9999
    min_angle = 0
    for i in range(len(ground_truth_measurements[index])):
        shifted_graph = np.roll(ground_truth_measurements[index],i)
        error = find_sum_of_squares_error(np.array(shifted_graph),np.array(measured))
        if error < min_error:
            min_error = error
            min_angle = i*3.0/pi
    if min_angle > pi:
        min_angle -= 2*pi
    return min_angle