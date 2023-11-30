import matplotlib.pyplot as plt
import numpy as np
import pygad

import math
from copy import deepcopy
import random
import sys


def clamp(mi,val,ma) : return max(mi, min(val, ma))

# fuzzy logic operators
def prod_tnorm(*vals): # AND
	tot = 1
	for val in vals : tot *= val
	return tot
def prod_tconorm(*vals): # OR
	l = 0
	for r in vals:
		l = l+r - l*r
	return l

# fuzzy membership sets
class Set:
	def __init__(self, w1,w2,tag, sat=[False,False]):
		self.w1 = w1 ; self.w2 = w2
		self.sat = sat
		self.tag = tag
		
		self.val = 0
	
	def fuzzify(self, val) : self.val = self.membership(val)
	
	@staticmethod
	def plotAll(fsets, premise=1):
		data = []
		for fset in fsets : data += fset.trace(premise)
		
		plt.figure()
		plt.plot(*data)
class SetTrig(Set):
	def __init__(self, b,l, *args,**kwargs):
		Set.__init__(self, *args,**kwargs)
		self.b = b
		self.l = l
	
	def membership(self, val):
		if self.sat[1] and val > self.w2 : return 1
		if self.sat[0] and val < self.w1 : return 1
		if val > self.b+self.l/2 or val < self.b-self.l/2 : return 0
		return 1 - abs(val - self.b) / (self.l/2)
	def area(self, premise):
		return premise * self.l /2
	
	# returns x,y points
	def trace(self, premise=1):
		x = []
		
		if self.sat[0] : x.append(self.w1*3/2)
		x += [self.b-self.l/2,self.b,self.b+self.l/2]
		if self.sat[1] : x.append(self.w2*3/2)
		
		y = [self.membership(val)*premise for val in x]
		
		return [x,y]
	
	@staticmethod
	def autoN(n, w1,w2, sat, *args,**kwargs):
		return [SetTrig( w1+i*(w2-w1)/(n-1), 2*(w2-w1)/(n-1), w1,w2, sat=([i==0,i==n-1] if sat else [False,False]), *args,**kwargs ) for i in range(n)]
class SetGauss(Set):
	def __init__(self, c,std, *args,**kwargs):
		Set.__init__(self, *args,**kwargs)
		self.b = c
		self.std = std
	
	def membership(self, val):
		if self.sat[1] and val > self.w2 : return 1
		if self.sat[0] and val < self.w1 : return 1
		
		return math.exp( -1/2 * ((val-self.b) / self.std)**2 )
	def area(self, premise):
		return premise * self.std*math.sqrt(2*math.pi)
	
	def trace(self, premise=1):
		N = 100
		x = [self.b-self.std*4 + 8*self.std*i/N for i in range(N)]
		y = [self.membership(val)*premise for val in x]
		
		return [x,y]
	
	# automatically generate n sets spread uniformly over [w1,w2] range
	@staticmethod
	def autoN(n, w1,w2, std, sat, *args,**kwargs):
		return [SetGauss( w1+i*(w2-w1)/(n-1), std, w1,w2, sat=([i==0,i==n-1] if sat else [False,False]), *args,**kwargs ) for i in range(n)]
	@staticmethod
	def autoN_cn(n, sat, *args,**kwargs): # centered 0, normalized autoN
		return SetGauss.autoN(n, -1,1, 1/n, sat, *args,**kwargs)
	@staticmethod
	def autoN_ln(n, sat, *args,**kwargs): # left 0, normalized autoN
		return SetGauss.autoN(n, 0,1, 0.5/n, sat, *args,**kwargs)

# fuzzy conditions
class Cond:
	def __init__(self, objs, op):
		self.objs = objs
		self.op = op
	
	# return list of fuzzy sets used in this condition
	def sets(self):
		tmp = []
		for obj in self.objs:
			if isinstance(obj,Cond) : tmp += obj.sets()
			else                    : tmp += [obj]
		return tmp
	# get logic value of this condition (fuzzify should be called on the rule first)
	def value(self):
		return self.op(*[obj.value() if isinstance(obj,Cond) else obj.val for obj in self.objs])
class AND(Cond):
	def __init__(self, *objs, **kwargs):
		Cond.__init__(self, objs, prod_tnorm, **kwargs)
class OR(Cond):
	def __init__(self, *objs, **kwargs):
		Cond.__init__(self, objs, prod_tconorm, **kwargs)

# defuzzification methods
def CA(rules):
	out = 0
	totPremise = 0
	for rule in rules:
		fset = rule.fset_out
		premise = rule.premise
		
		out += fset.b*premise
		totPremise += premise
	
	return out / totPremise
def COG(rules):
	out = 0
	totArea = 0
	for rule in rules:
		fset = rule.fset_out
		premise = rule.premise
		
		area = fset.area(premise)
		out += area*fset.b
		totArea += area
	
	return out / max(1e-9,totArea)

# fuzzy rule (one output variable per rule)
class Rule:
	def __init__(self, cond, fset_out):
		self.cond = cond
		self.premise = 0
		
		# sort sets in their categories
		self.fsets_in = {fset.tag: fset for fset in self.cond.sets()}
		self.fset_out = fset_out
	
	def fuzzify(self):
		self.premise = self.cond.value()
	@staticmethod
	def fuzzifyAll(rules):
		for rule in rules : rule.fuzzify()

# MIMO fuzzy system
class System:
	def __init__(self, rules):
		self.rules = rules
		
		# map output tag to rules
		self.rulemap = {}
		for rule in rules:
			tag = rule.fset_out.tag
			if not tag in self.rulemap : self.rulemap[tag] = []
			self.rulemap[tag].append(rule)
		# map input tag to input set
		self.fsets_in = {}
		for rule in rules:
			for tag in rule.fsets_in:
				fset = rule.fsets_in[tag]
				if not tag in self.fsets_in : self.fsets_in[tag] = []
				if not fset in self.fsets_in[tag] : self.fsets_in[tag].append(fset)
		# map output tag to output sets
		self.fsets_out = {}
		for rule in rules:
			tag = rule.fset_out.tag
			if not tag in self.fsets_out : self.fsets_out[tag] = []
			if not rule.fset_out in self.fsets_out[tag] : self.fsets_out[tag].append(rule.fset_out)
		# map output tag to corresponding input sets
		self.fsets_outin = {tag:[] for tag in self.rulemap}
		for rule in rules:
				tag = rule.fset_out.tag
				for fset in rule.fsets_in.values():
					if not fset in self.fsets_outin[tag] : self.fsets_outin[tag].append(fset)
	
	def input(self, valmap):
		for tag in valmap:
			for fset in self.fsets_in[tag] : fset.fuzzify(valmap[tag])
		Rule.fuzzifyAll(self.rules)
	def compute(self, valmap, tag=None,method=COG):
		self.input(valmap)
		
		if tag is None : return {tag: method(self.rulemap[tag]) for tag in self.rulemap}
		else           : return method(self.rulemap[tag])
	
	# adjust paramters from training data using gradient descent
	# prerequesites: non-saturated input gaussians, AND rules, CA defuzzification, using * as logic AND / combinator
	def gradient_analytical(self, data, lamb=0.5):
		# data : [input, output] <=> [{'tagx1':x1, 'tagx2':x2, ...}, {'tagy1':y1, 'tay2':y2, ...}]
		
		eps = 1e-3
		errtot = 0
		
		# for each output variable
		for tag in self.rulemap:
			rules = self.rulemap[tag]
			
			fk = self.compute(data[0],tag=tag,method=CA)
			err = fk - data[1][tag]
			errfull = 1/2*err**2
			
			while errfull > eps:
				
				smui = 0
				for rule in rules : smui += rule.premise
				
				# for each rule concerned with the current output variable
				for rule in rules:
					
					# update output center
					b = rule.fset_out.b
					rule.fset_out.b -= lamb*err * rule.premise/smui
					
					# update input gaussian paramters
					for fset in rule.fsets_in.values():
						c = fset.b
						sig = fset.std
						
						tmp = (b-fk)/smui * rule.premise
						fset.b -= lamb*err * tmp * ((data[0][fset.tag]-c))/(sig**2)
						fset.std -= lamb*err * tmp * ((data[0][fset.tag]-c)**2)/(sig**3)
				
				fk = self.compute(data[0],tag=tag,method=CA)
				err = fk - data[1][tag]
				errfull = 1/2*err**2
				
			errtot += errfull
		
		return errtot
	# adjust paramters from training data using gradient method with finite differences
	# no prerequesites
	def _gradient_finite(self, fset, params, h, lamb, indata,outdata, tag):
		for p in params:
			val = getattr(fset,p)
			
			setattr(fset,p, val+h/2)
			err_p = 1/2 * (self.compute(indata,tag=tag) - outdata[tag])**2
			setattr(fset,p, val-h/2)
			err_m = 1/2 * (self.compute(indata,tag=tag) - outdata[tag])**2
			
			derr = (err_p - err_m) / h
			setattr(fset,p, val - lamb*derr)
	def gradient_finite(self, data, params_in=['b'], params_out=['b'], lamb=0.5):
		# data : [input, output] <=> [{'tagx1':x1, 'tagx2':x2, ...}, {'tagy1':y1, 'tay2':y2, ...}]
		
		eps = 1e-3
		h = 1e-3
		errtot = 0
		
		for tag in self.fsets_outin:
			fk = self.compute(data[0], tag=tag)
			errfull = 1/2 * (fk-data[1][tag])**2
			
			while errfull > eps:
				# for each output set corresponding with current output tag
				for fset in self.fsets_out[tag] : self._gradient_finite(fset, params_out, h, lamb, data[0],data[1], tag)
				# for each input set concerned with current output tag
				for fset in self.fsets_outin[tag] : self._gradient_finite(fset, params_in, h, lamb, data[0],data[1], tag)
				
				fk = self.compute(data[0], tag=tag)
				errfull = 1/2 * (fk-data[1][tag])**2
				
			errtot += errfull
		
		return errtot
	
	def train(self, datas, method, **kwargs):
		eps = 1e-4
		lastToterr = 9999
		while True:
			toterr = 0
			for data in datas : toterr += method(self, data, **kwargs)
			if abs(lastToterr-toterr) <= eps : return toterr
			
			lastToterr = toterr

# Genetic algorithm using PyGAD (using gaussian sets)
class Population:
	def __init__(self, rules):
		self.system = System(rules)
	
	def init(self, nbiter,nbpop,fitness):
		population = []
		for i in range(nbpop):
			population.append([])
			
			# randomize system parameters
			for rule in self.system.rules:
				for tag in rule.fsets_in:
					population[-1].append( random.uniform(rule.fsets_in[tag].w1, rule.fsets_in[tag].w2) )
					population[-1].append( random.uniform(rule.fsets_in[tag].w1, rule.fsets_in[tag].w2) )
				population[-1].append( random.uniform(rule.fset_out.w1, rule.fset_out.w2) )
				population[-1].append( random.uniform(rule.fset_out.w1, rule.fset_out.w2) )
		
		def progress(ga_inst):
			print(f"Population {ga_inst.generations_completed}/{ga_inst.num_generations} :: fitness = {ga_inst.best_solution()[1]}")
		
		self.ga_inst = pygad.GA(
			num_generations=nbiter,
			num_parents_mating=round(nbpop/2),
			initial_population=np.array(population),
			fitness_func=lambda ga_inst,solution,solution_idx : self.chromosomeFitness(fitness, solution),
			on_generation=progress
		)
	def evolve(self):
		self.ga_inst.run()
		self.ga_inst.plot_fitness()
		self.ga_inst.save(filename="ga_data")
	
	def chromosomeFitness(self, fun, chromosome):
		# update system from chromosome
		i=0
		for rule in self.system.rules:
			for fset in rule.fsets_in.values():
				fset.b   =           clamp(fset.w1, chromosome[i], fset.w2)  ;i+=1
				fset.std = max(1e-9, clamp(fset.w1, chromosome[i], fset.w2)) ;i+=1
				
			rule.fset_out.b   =           clamp(rule.fset_out.w1, chromosome[i], rule.fset_out.w2)  ;i+=1
			rule.fset_out.std = max(1e-9, clamp(rule.fset_out.w1, chromosome[i], rule.fset_out.w2)) ;i+=1
		
		# compute fitness using new system
		return fun()

# Genetic algorithm (using gaussian sets)
#class Population:
#	def __init__(self, rules):
#		self.system = System(rules)
#		self.fxdpnt = 10**3
#		self.nbc = 10
#		
#	# randomize population
#	def init(self, nbentities):
#		if nbentities % 2 != 0 :
#			print("ERROR :: must have an even number for the population")
#			sys.exit(0)	
#		
#		self.chromosomes = []
#		for i in range(nbentities):
#			# randomize system parameters
#			for rule in self.system.rules:
#				for tag in rule.fsets_in:
#					rule.fsets_in[tag].b = random.uniform(rule.fsets_in[tag].w1, rule.fsets_in[tag].w2)
#					rule.fsets_in[tag].std = random.uniform(rule.fsets_in[tag].w1, rule.fsets_in[tag].w2)
#				rule.fset_out.b = random.uniform(rule.fset_out.w1, rule.fset_out.w2)
#				rule.fset_out.std = random.uniform(rule.fset_out.w1, rule.fset_out.w2)
#			# convert to chromosome data
#			self.chromosomes.append(self.chromosomeFromSystem(self.system))
#	
#	def floatToFxdstr(self, val) : return str(round(val*self.fxdpnt)).zfill(self.nbc)[:self.nbc]
#	def fxdstrToFloat(self, val) : return int(val)/self.fxdpnt
#	
#	def chromosomeFromSystem(self, system):
#		chromosome = ''
#		for rule in system.rules:
#			for fset in rule.fsets_in.values():
#				chromosome += self.floatToFxdstr(fset.b)
#				chromosome += self.floatToFxdstr(fset.std)
#			chromosome += self.floatToFxdstr(rule.fset_out.b)
#			chromosome += self.floatToFxdstr(rule.fset_out.std)
#		return chromosome
#	def systemFromChromosome(self, system, chromosome):
#		i = 0
#		for rule in system.rules:
#			for fset in rule.fsets_in.values():
#				fset.b = clamp( fset.w1, self.fxdstrToFloat(chromosome[i:i+self.nbc]), fset.w2 )   ; i += self.nbc
#				fset.std = clamp( fset.w1, self.fxdstrToFloat(chromosome[i:i+self.nbc]), fset.w2 ) ; i += self.nbc
#				
#			rule.fset_out.b = clamp( rule.fset_out.w1, self.fxdstrToFloat(chromosome[i:i+self.nbc]), rule.fset_out.w2 )   ; i += self.nbc
#			rule.fset_out.std = clamp( rule.fset_out.w1, self.fxdstrToFloat(chromosome[i:i+self.nbc]), rule.fset_out.w2 ) ; i += self.nbc
#	
#	def computeFitness(self, fitness):
#		fitres = []
#		for chromosome in self.chromosomes:
#			self.systemFromChromosome(self.system, chromosome)
#			fitres.append(fitness(self.system))
#		return fitres
#	def generation(self, fitness):
#		fitres = self.computeFitness(fitness)
#		fittot = sum(fitres)
#		probs = [fitval/fittot for fitval in fitres]
#		
#		matingpool = random.choices(self.chromosomes, weights=probs, k=len(self.chromosomes))
#		random.shuffle(matingpool)
#		
#		# clear chromosomes
#		self.chromosomes = []
#		
#		pairs = [matingpool[i:i+2] for i in range(0,len(matingpool),2)]
#		
#		prob_crossover = 0.95
#		for pair in pairs:
#			chromosome0 = pair[0] ; chromosome1 = pair[1]
#			
#			# crossover
#			if random.uniform(0,1) <= prob_crossover:
#				crosssite = random.randint(0, len(chromosome0))
#				nchromosome0 = chromosome0[:crosssite-1] + chromosome1[crosssite:]
#				nchromosome1 = chromosome1[:crosssite-1] + chromosome0[crosssite:]
#			else:
#				nchromosome0 = chromosome0
#				nchromosome1 = chromosome1
#				
#			self.chromosomes += [nchromosome0, nchromosome1]
#			
#		return fitres
#	
#	def evolve(self, fitness, maxgens):
#		maxfits = []
#		for i in range(maxgens):
#			# pre-reproduction fitness
#			maxfit = max(self.generation(fitness))
#			maxfits.append(maxfit)
#			print(f"Population {i}/{maxgens} :: max fit = {maxfit}")
#
