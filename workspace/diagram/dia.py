#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
filename = 'points.txt'
xpoints = []
ypoints = []
def main():
  with open(filename,'rw') as f:
    for line in f:
      a = [float(n) for n in line.strip().split(' ')]
      print str(a[0])+ "   " +  str(a[1])
      xpoints.append(a[0])
      ypoints.append(a[1])

  print "Odczytano punkty z pliku\n" 
  #print mypoints
#for pair in mypoints:
#  try:
#    x,y = pair[0],pair[1]
#    print "Oto punkt x " + x
#  except IndexError:
#    print "A line is the file doesn't have enough entries."
  #points = [[2,4],[2,8],[4,6],[6,8]]
  #plt.figure(figsize=(3, 3))
  #plt.plot([100,200,300,400,-200],[0.1,0.2,0.8,0.9,-1.0])
  #plt.Polygon(mypoints, closed=None, fill=None, edgecolor='r')
  plt.plot(xpoints, ypoints)
  plt.savefig('myplot.png')
  plt.close()
  #line.savefig('a.png')
  #line.close()
if __name__ == '__main__':
  main()

