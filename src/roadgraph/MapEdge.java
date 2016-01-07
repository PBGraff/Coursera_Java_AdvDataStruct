package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint from, to;
	private String roadName, roadType;
	private double length;
	
	MapEdge(GeographicPoint from, GeographicPoint to, String name, String type, double len) {
		this.from = from;
		this.to= to;
		this.roadName = name;
		this.roadType = type;
		this.length = len;
	}
	
	public GeographicPoint getStart() {
		return from;
	}
	
	public GeographicPoint getEnd() {
		return to;
	}
	
	public double getLength() {
		return length;
	}
	
	public String getName() {
		return roadName;
	}
	
	public String getType() {
		return roadType;
	}
	
	public int hashCode() {
		return from.hashCode()+to.hashCode();
	}
}
