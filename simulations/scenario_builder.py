import os
import numpy as np

networks = ['dmat.net.xml', 'dpt.net.xml', 'rclt.net.xml', 'rio450.net.xml', 'ybt.net.xml']

routesEnEx = ['DMATEntranceExit', 'DPTEntranceExit', 'RCLTEntranceExit', 'RIO450EntranceExit', 'YBTEntranceExit']
routesExEn = ['DMATExitEntrance', 'DPTExitEntrance', 'RCLTExitEntrance','', 'YBTExitEntrance']

edgesEnEx = ['8916170 396959336 59764940 28412306 33764603 23874453 59763339 59763602 59763350 59763232 59763351',
			 '100655411 22328500 22328499 4881656 4937319 4937318 129662536 4384688 55691860 4383362 4383363',
			 '-27050852 26938354 85644944 87297166 26938250 8921173 -87376660#2 -87376660#1 -87376660#0 -87376695 -87376682',
			 '372924399#0 372924399#1 372924399#2 372924399#3 372924399#4 372924399#5 372924399#6 372924399#7 30989127#0 30989127#1 65583216#0 65583216#1 130347317#0 130347317#1 130347317#2 130347317#3 130347317#4 130347317#5 130347317#6 130347317#7 130347317#8 130347317#9 130347317#10 130347317#11 130347319 30989136 251211066 173419300 252028893 422049769 173419316 173419311 176407075 194996847',
			 '50690291 179235221 52721870 50691047 52527662 237731428#0 237731428#1 237731428#2']

edgesExEn = ['161493573 59766629 59766617 59766616 59766615 59766608 59766570 59766528 399402797 396959337 59766456',
			 '335075000 4383949 106130118 334873781 4383747 223780739 326974422 110935782 23007012 4384666 4937321 4937320 48290563',
			 '87376682 87376695 87376660#0 87376660#1 87376660#2 26938247 133376103 8921175 26938401 87297169 85644934 27050852',
	 		 '',
			 '236348361 236348360#0 236348360#1 236348360#2 236348364#0 236348364#1 11415208 23874736 394443191']

maxspeeds = ['10.5', '11.11', '8.89', '13.0', '11.15']

periods = ['1', '3', '5', '10', '15']

stepTime = '0.1'

beginTime = '0'
endTime = '7200'


for  i in range(0,len(networks)):
	
	for period in periods:
		
		route = networks[i].split('.')[0]+'-'+str(period)+'.rou.xml'
		launchd = networks[i].split('.')[0]+'-'+str(period)+'.launchd.xml'
		cfg = networks[i].split('.')[0]+'-'+str(period)+'.sumo.cfg'

		with open(cfg,'w') as cfgFile:
			cfgFile.write('<?xml version="1.0"?>\n')
			cfgFile.write('<configuration>\n')
			cfgFile.write('\t<input>\n')
			cfgFile.write('\t\t<net-file value="'+networks[i]+'" />\n')
			cfgFile.write('\t\t<route-files value="'+route+'" />\n')
			cfgFile.write('\t</input>\n')
			cfgFile.write('\t<time>\n')
			cfgFile.write('\t\t<begin value="'+beginTime+'" />\n')
			cfgFile.write('\t\t<end value="'+endTime+'" />\n')
			cfgFile.write('\t</time>\n')
			cfgFile.write('\t<step-length value="'+stepTime+'" />\n')
			cfgFile.write('\t<routing-algorithm value="CHWrapper" />\n')
			cfgFile.write('\t<time-to-teleport value="-1" />\n')
			cfgFile.write('</configuration>\n')
		cfgFile.close()	

		with open(route,'w') as routeFile:
			routeFile.write('<?xml version="1.0" encoding="UTF-8"?>\n')
			routeFile.write('<routes>\n')
			routeFile.write('\t<vType accel="1.0" decel="5.0" id="Car" length="3.5" maxSpeed="'+maxspeeds[i]+'" sigma="0.5" />\n')
			routeFile.write('\t<route id="'+routesEnEx[i]+'" edges="'+edgesEnEx[i]+'"/>\n')
			
			if(networks[i] != 'rio450.net.xml'):
				routeFile.write('\t<route id="'+routesExEn[i]+'" edges="'+edgesExEn[i]+'"/>\n')			

			routeFile.write('\t<flow id="flowEntranceExit" departLane="random" type="Car" route="'+routesEnEx[i]+'" begin="0.0" departSpeed="max" period="'+period+'" number="1000" />\n')
			
			if(networks[i] != 'rio450.net.xml'):
				routeFile.write('\t<flow id="flowExitEntrance" departLane="random" type="Car" route="'+routesExEn[i]+'" begin="0.0" departSpeed="max" period="'+period+'" number="1000" />\n')
			
			routeFile.write('</routes>\n')
		routeFile.close()

		with open(launchd,'w') as launchdFile:
			launchdFile.write('<?xml version="1.0"?>\n')
			launchdFile.write('<!-- debug config -->\n')
			launchdFile.write('\t<launch>\n')
			launchdFile.write('\t\t<copy file="'+networks[i]+'" />\n')
			launchdFile.write('\t\t<copy file="'+route+'" />\n')
			launchdFile.write('\t\t<!--<copy file="erlangen.poly.xml" /> -->\n')
			launchdFile.write('\t\t<copy file="'+cfg+'" type="config" />\n')
			launchdFile.write('\t</launch>\n')
		launchdFile.close()
