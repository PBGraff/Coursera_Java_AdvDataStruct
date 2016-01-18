package roadgraph;

public class MapEdge {
	private MapNode from, to;
	private String roadName, roadType;
	private double length;
	
	MapEdge(MapNode from, MapNode to, String name, String type, double len) {
		this.from = from;
		this.to= to;
		this.roadName = name;
		this.roadType = type;
		this.length = len;
	}
	
	public MapNode getStart() {
		return from;
	}
	
	public MapNode getEnd() {
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
	
//	public int hashCode() {
//		return from.hashCode()+to.hashCode();
//	}
}
