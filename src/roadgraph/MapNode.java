package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

public class MapNode implements Comparable<Object> {
	private GeographicPoint location;
	private Set<MapEdge> neighbors;
	
	MapNode(GeographicPoint p) {
		this.location = p;
		this.neighbors = new HashSet<MapEdge>();
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
	
	public int hashCode() {
		return location.hashCode();
	}

	@Override
	public int compareTo(Object o) {
		return (new Integer(this.hashCode()).compareTo(o.hashCode()));
	}
}
