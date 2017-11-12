import numpy as np
import matplotlib.pyplot as plt


colors = ['FFBF7F' '00D0B5' '7853A8' '63A5C3' '0081FF' 'FF3399' 'CF0060' 'FFFFFF' 'FF6699' '8F5873' 'F26D7D' '6666CC' 'F58345' 'FFFF66' '99FF99' '12E3DB']

colors = ['r','b']
count = 0


filePath = "/local1/liborio/workspace/CoopPos/localization/Graphs/out_without_id/DPTEntranceExit.txt"
listx, listy = np.loadtxt(filePath, delimiter='\t', unpack = True)
for i in range(2, len(listx), 2):
	#plt.plot(listx[i-2:i], listy[i-2:i], color = colors[count],linewidth=3,zorder=1)#,s =30)

	plt.scatter(listx[i-2:i], listy[i-2:i],color = colors[count],linewidth=1, zorder=2) #s =30,
	plt.text(listx[i-2], listy[i-2], str(i-2))
	plt.text(listx[i-1], listy[i-1], str(i-1))

	if count == 0:
		count = 1
	else:
		if count == 1:
			count = 0

plt.xlabel("UTM (x)")
#plt.xticks(np.arange(1, 1000, 10))
#plt.xscale('log')
#plt.yscale('log')
plt.ylabel("UTM (y)")
plt.legend(loc='upper left', bbox_to_anchor=(0.01, 1.016), fancybox=True, shadow=True, fontsize='small')

fig = plt.gcf()
#fig.suptitle('Error Per Cycle',y=0.98)
plt.show()
plt.draw()
plt.grid(True)
fig.savefig(tunnel+'.png',dpi=200)

#tunnels = ['RIO450', 'DPT', 'DMAT', 'RCLT' , 'YBT']
# colors = ['g', 'c', 'm', 'k' , 'b']

# ways = ['EntranceExit', 'ExitEntrance']

# for tunnel in tunnels:

# 	for way in ways:
# 		if(way == "ExitEntrance" and tunnel == "RIO450"):
# 			continue
		
# 		filePath = "out_without_id/"+tunnel+way+".txt"
# 		listx, listy = np.loadtxt(filePath, delimiter='\t', unpack = True)
# 		plt.plot(listx, listy,label=tunnel, color = colors[count],linewidth=3,zorder=1)#,s =30)
# 		plt.scatter(listx, listy,color = 'k',linewidth=3, zorder=2) #s =30,

# 		filePathGPS = "/home/liborio/VehicularNetworking/workspace/Projection/AllOutages_AllPointsxy/"+tunnel+way+".txt"
# 		listxGPS, listyGPS = np.loadtxt(filePathGPS, delimiter='\t', unpack = True, skiprows=1, usecols=(1,2))
# 		if(way == ways[0]):
# 			plt.scatter(listxGPS[30:40], listyGPS[30:40], label='GPS', c='r',zorder=3)
# 		else:
# 			plt.scatter(listxGPS[30:40], listyGPS[30:40], label='GPS', c='b',zorder=3)

# 	plt.xlabel("Longitude (x)")
# 	#plt.xticks(np.arange(1, 1000, 10))
# 	#plt.xscale('log')
# 	#plt.yscale('log')
# 	plt.ylabel("Latitude (y)")
# 	plt.legend(loc='upper left', bbox_to_anchor=(0.01, 1.016), fancybox=True, shadow=True, fontsize='small')

# 	fig = plt.gcf()
# 	#fig.suptitle('Error Per Cycle',y=0.98)
# 	plt.show()
# 	plt.draw()
# 	plt.grid(True)
# 	fig.savefig(tunnel+'.png',dpi=200)
# 	count+=1

	


