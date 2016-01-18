package roadgraph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import geography.GeographicPoint;

public class MapNode implements Comparable<Object> {
	private GeographicPoint location;
	private Set<MapEdge> neighbors;
	private double distFromStart, estDistFromGoal;
	private Map<GeographicPoint,List<GeographicPoint>> shortestPaths;
	
	MapNode(GeographicPoint p) {
		this.location = p;
		this.neighbors = new HashSet<MapEdge>();
		this.distFromStart = Double.POSITIVE_INFINITY;
		this.estDistFromGoal = Double.POSITIVE_INFINITY;
		this.shortestPaths = new HashMap<GeographicPoint,List<GeographicPoint>>();
	}
	
	public void addEdge(MapEdge e) {
		neighbors.add(e);
	}
	
	public void addEdge(MapNode to, String name, String type, double len) {
		MapEdge e = new MapEdge(this, to, name, type, len);
		addEdge(e);
	}
	
	public GeographicPoint getLocation() {
		return location;
	}
	
	public Set<MapEdge> getNeighbors() {
		return neighbors;
	}
	
	public int getNumNeighbors() {
		return neighbors.size();
	}
	
	public void setDistanceFromStart(double d) {
		this.distFromStart = d;
	}
	
	public double getDistanceFromStart() {
		return distFromStart;
	}
	
	public void setEstDistFromGoal(double d) {
		this.estDistFromGoal = d;
	}
	
	public double getEstDistFromGoal() {
		return estDistFromGoal;
	}
	
	public double getTotalDistance() {
		return distFromStart + estDistFromGoal;
	}
	
	public boolean hasPath(GeographicPoint dest) {
		return this.shortestPaths.containsKey(dest);
	}
	
	public List<GeographicPoint> getPath(GeographicPoint dest) {
		return this.shortestPaths.get(dest);
	}
	
	public void setPath(GeographicPoint dest, List<GeographicPoint> path) {
		this.shortestPaths.put(dest, path);
	}
	
//	public int hashCode() {
//		return location.hashCode();
//	}

	@Override
	public int compareTo(Object o) {
		return (new Integer(this.hashCode()).compareTo(o.hashCode()));
	}
}
