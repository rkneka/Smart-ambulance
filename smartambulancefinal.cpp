#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const double R_EARTH_KM = 6371.0;
double deg2rad(double deg) { return deg * M_PI / 180.0; }

double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a = sin(dlat/2)*sin(dlat/2) +
               cos(deg2rad(lat1))*cos(deg2rad(lat2))*
               sin(dlon/2)*sin(dlon/2);
    double c = 2*atan2(sqrt(a), sqrt(1-a));
    return R_EARTH_KM*c;
}

/* --------------------------- Structures --------------------------- */
struct Node { string name; double lat, lon; };
struct Edge { int to; double distance; bool hasSignal; };
struct Ambulance { int id; int nodeIndex; bool isAvailable; };
struct TrafficSignal { bool isRed = true; };
struct PatrolEvent { int ambulanceId; int signalKey; };
struct Incident {
    int id;
    int accidentNode;
    Ambulance* assignedAmbulance;
    int hospitalNode;
    vector<int> path;
};

/* --------------------------- City Class --------------------------- */
class City {
    vector<Node> nodes;
    vector<vector<Edge>> adj;
    vector<int> hospitals;
    vector<Ambulance> ambulances;
    unordered_map<int, TrafficSignal> signals;   // key = min(u,v)*100 + max(u,v)
    queue<PatrolEvent> patrolQueue;

public:
    City() {}

    string getNodeName(int idx) const {
        return (idx>=0 && idx<(int)nodes.size()) ? nodes[idx].name : "Unknown";
    }

    bool loadNodes(const string& fn) {
        ifstream f(fn);
        if (!f) { cerr<<"[ERROR] Cannot open "<<fn<<endl; return false; }
        string line;
        while (getline(f,line)) {
            if (line.empty() || line[0]=='#') continue;
            stringstream ss(line);
            string name; double lat,lon;
            ss>>name>>lat>>lon;
            if (ss.fail()) continue;
            nodes.push_back({name,lat,lon});
            adj.resize(nodes.size());
        }
        return true;
    }

    bool loadRoads(const string& fn) {
        ifstream f(fn);
        if (!f) { cerr<<"[ERROR] Cannot open "<<fn<<endl; return false; }
        string line;
        while (getline(f,line)) {
            if (line.empty() || line[0]=='#') continue;
            stringstream ss(line);
            int u,v,hasSig;
            ss>>u>>v>>hasSig;
            if (ss.fail() || u<0||v<0||u>=(int)nodes.size()||v>=(int)nodes.size()) continue;
            double d = haversine(nodes[u].lat,nodes[u].lon,nodes[v].lat,nodes[v].lon);
            Edge e1{v,d,hasSig==1};
            Edge e2{u,d,hasSig==1};
            adj[u].push_back(e1);
            adj[v].push_back(e2);
            if (hasSig) {
                int key = min(u,v)*100 + max(u,v);
                signals[key] = TrafficSignal{};
            }
        }
        return true;
    }

    void addHospital(int idx) { if (idx>=0 && idx<(int)nodes.size()) hospitals.push_back(idx); }
    void addAmbulance(int id,int idx) { ambulances.push_back({id,idx,true}); }

    void showMap() const {
        cout<<fixed<<setprecision(4);
        cout<<"\n--- CITY MAP ---\n";
        for (size_t i=0;i<nodes.size();++i) {
            cout<<i<<": "<<nodes[i].name<<" ("<<nodes[i].lat<<","<<nodes[i].lon<<")\n";
            cout<<"  Edges: ";
            for (const auto& e:adj[i]) {
                cout<<"("<<e.to<<":"<<getNodeName(e.to)<<","<<e.distance;
                if (e.hasSignal) cout<<",Signal";
                cout<<") ";
            }
            cout<<"\n";
        }
        cout<<"\nHospitals: ";
        for (int h:hospitals) cout<<h<<"["<<getNodeName(h)<<"] ";
        cout<<"\nAmbulances: ";
        for (const auto& a:ambulances)
            cout<<"[A"<<a.id<<"@"<<a.nodeIndex<<":"<<getNodeName(a.nodeIndex)<<"] ";
        cout<<"\n";
    }

    vector<int> shortestPath(int src,int dest,double& totalDist) const {
        int n = (int)nodes.size();
        vector<double> dist(n,1e9);
        vector<int> parent(n,-1);
        using PDI = pair<double,int>;
        priority_queue<PDI,vector<PDI>,greater<PDI>> pq;
        dist[src]=0; pq.push({0,src});

        while (!pq.empty()) {
            PDI front = pq.top(); pq.pop();
            double d = front.first;
            int u   = front.second;
            if (d > dist[u]) continue;
            if (u==dest) break;
            for (const auto& e:adj[u]) {
                int v = e.to;
                double nd = d + e.distance;
                if (nd < dist[v]) {
                    dist[v]=nd;
                    parent[v]=u;
                    pq.push({nd,v});
                }
            }
        }
        totalDist = dist[dest];
        vector<int> path;
        if (totalDist>1e8) return path;
        for (int at=dest;at!=-1;at=parent[at]) path.insert(path.begin(),at);
        return path;
    }

    Ambulance* findNearestAmbulance(int accidentNode,double& distKm) {
        double bestD = 1e9; Ambulance* best = nullptr;
        for (auto& a:ambulances) {
            if (!a.isAvailable) continue;
            double d;
            shortestPath(a.nodeIndex,accidentNode,d);
            if (d<bestD) { bestD=d; best=&a; }
        }
        if (best) {
            cout<<"Nearest ambulance: A"<<best->id
                <<" at "<<getNodeName(best->nodeIndex)
                <<" (Distance: "<<fixed<<setprecision(3)<<bestD<<" km)\n";
            distKm = bestD;
        } else cout<<"No available ambulance!\n";
        return best;
    }

    int findNearestHospital(int srcNode,double& distKm) const {
        double bestD = 1e9; int best = -1;
        for (int h:hospitals) {
            double d;
            shortestPath(srcNode,h,d);
            if (d<bestD) { bestD=d; best=h; }
        }
        if (best!=-1) {
            cout<<"Nearest hospital: "<<getNodeName(best)
                <<" (Distance: "<<fixed<<setprecision(3)<<bestD<<" km)\n";
            distKm = bestD;
        } else cout<<"No reachable hospital!\n";
        return best;
    }

    void setSignalGreen(int key, int ambId) {
        if (signals.find(key) == signals.end()) return;
        if (signals[key].isRed) {
            signals[key].isRed = false;
            int u = key / 100, v = key % 100;
            cout << "[Signal] " << getNodeName(u) << "-" << getNodeName(v) << " -> GREEN\n";
        }
        for (auto& p : signals) {
            if (p.first == key) continue;
            if (!p.second.isRed) {
                p.second.isRed = true;
                int uu = p.first / 100, vv = p.first % 100;
                cout << "[Signal] " << getNodeName(uu) << "-" << getNodeName(vv) << " -> RED\n";
            }
        }
        patrolQueue.push({ambId, key});
    }

    void processPatrolQueue() {
        while (!patrolQueue.empty()) {
            PatrolEvent e = patrolQueue.front(); patrolQueue.pop();
            int u = e.signalKey / 100, v = e.signalKey % 100;
            cout << " Patrol notified for ambulance A" << e.ambulanceId
                 << " at signal (" << getNodeName(u) << "-" << getNodeName(v) << ")\n";
        }
    }

    void moveAmbulanceFull(Ambulance* a, vector<int>& path) {
        if (!a || path.empty()) {
            cout << "Ambulance already at destination or invalid!\n";
            return;
        }
        for (size_t step = 0; step + 1 < path.size(); ++step) {
            int u = path[step], v = path[step+1];
            int key = min(u,v)*100 + max(u,v);
            setSignalGreen(key, a->id);
            a->nodeIndex = v;
            cout << "Ambulance A" << a->id << " moved to " << getNodeName(v) << "\n";
            processPatrolQueue();
        }
    }

    void showGoogleMapsLink(const vector<int>& path) const {
        if (path.size()<2) return;
        cout<<"\nGoogle Maps Route: https://www.google.com/maps/dir/?api=1";
        cout<<"&origin="<<nodes[path.front()].lat<<","<<nodes[path.front()].lon;
        cout<<"&destination="<<nodes.back().lat<<","<<nodes.back().lon<<"\n";
    }
};

/* --------------------------- MAIN --------------------------- */
int main() {
    City city;
    if (!city.loadNodes("nodes.txt") || !city.loadRoads("roads.txt")) return 1;

    // Hospitals
    city.addHospital(7);   // Anna_Nagar
    city.addHospital(9);   // Mylapore
    city.addHospital(16);  // St_Thomas_Mount

    // Ambulances
    city.addAmbulance(1, 0);   // A1 @ Tambaram
    city.addAmbulance(2, 6);   // A2 @ T_Nagar
    city.addAmbulance(3, 12);  // A3 @ Sholinganallur

    vector<Incident> incidents;
    int incidentCounter = 1;

    while (true) {
        cout<<"\n==== SMART AMBULANCE MENU ====\n";

        // Show active incidents
        if (!incidents.empty()) {
            cout << "Active Incidents:\n";
            for (auto &inc : incidents) {
                cout << "ID " << inc.id << " at " << city.getNodeName(inc.accidentNode);
                if (inc.assignedAmbulance) cout << ", Ambulance A" << inc.assignedAmbulance->id;
                if (inc.hospitalNode != -1) cout << ", Hospital " << city.getNodeName(inc.hospitalNode);
                cout << "\n";
            }
        }

        cout<<"1. Show City Map\n";
        cout<<"2. Report Accident & Allocate Ambulance\n";
        cout<<"3. Assign Nearest Hospital\n";
        cout<<"4. Compute Shortest Path to Hospital\n";
        cout<<"5. Move Ambulance to Hospital (Full Path)\n";
        cout<<"6. Notify Hospital & Release Ambulance\n";
        cout<<"0. Exit\n";
        cout<<"Enter choice: "; 
        int ch; 
        cin >> ch;
        if (ch == 0) break;

        cout << "\n>>> You selected option " << ch << " <<<\n";

        switch(ch) {
            case 1:
                city.showMap();
                break;

            case 2: {
                int accidentNode;
                cout << "Enter accident node index: "; 
                cin >> accidentNode;
                double d;
                Ambulance* nearestAmb = city.findNearestAmbulance(accidentNode,d);
                if (!nearestAmb) break;
                nearestAmb->isAvailable = false;
                Incident inc;
                inc.id = incidentCounter++;
                inc.accidentNode = accidentNode;
                inc.assignedAmbulance = nearestAmb;
                inc.hospitalNode = -1;
                inc.path.clear();
                incidents.push_back(inc);
                break;
            }

            case 3: {
                if (incidents.empty()) { cout << "No incidents!\n"; break; }
                int selId;
                cout << "Select incident ID: "; cin >> selId;
                auto it = find_if(incidents.begin(), incidents.end(),
                                  [selId](const Incident &inc){ return inc.id == selId; });
                if (it == incidents.end()) { cout << "Invalid incident ID\n"; break; }
                double d;
                it->hospitalNode = city.findNearestHospital(it->accidentNode,d);
                break;
            }

            case 4: {
                if (incidents.empty()) { cout << "No incidents!\n"; break; }
                int selId;
                cout << "Select incident ID: "; cin >> selId;
                auto it = find_if(incidents.begin(), incidents.end(),
                                  [selId](const Incident &inc){ return inc.id == selId; });
                if (it == incidents.end()) { cout << "Invalid incident ID\n"; break; }
                if (!it->assignedAmbulance || it->hospitalNode == -1) { cout << "Assign ambulance and hospital first!\n"; break; }
                double totalD;
                it->path = city.shortestPath(it->assignedAmbulance->nodeIndex,it->hospitalNode,totalD);
                cout << "Shortest path: ";
                for (int n: it->path) cout << city.getNodeName(n) << " -> ";
                cout << "END\n";
                city.showGoogleMapsLink(it->path);
                break;
            }

            case 5: {
                if (incidents.empty()) { cout << "No incidents!\n"; break; }
                int selId;
                cout << "Select incident ID: "; cin >> selId;
                auto it = find_if(incidents.begin(), incidents.end(),
                                  [selId](const Incident &inc){ return inc.id == selId; });
                if (it == incidents.end()) { cout << "Invalid incident ID\n"; break; }
                if (!it->assignedAmbulance || it->path.empty()) { cout << "Compute path first!\n"; break; }
                city.moveAmbulanceFull(it->assignedAmbulance,it->path);
                break;
            }

            case 6: {
                if (incidents.empty()) { cout << "No incidents!\n"; break; }
                int selId;
                cout << "Select incident ID: "; cin >> selId;
                auto it = find_if(incidents.begin(), incidents.end(),
                                  [selId](const Incident &inc){ return inc.id == selId; });
                if (it == incidents.end()) { cout << "Invalid incident ID\n"; break; }
                if (it->hospitalNode != -1) cout << "Hospital " << city.getNodeName(it->hospitalNode) << " notified!\n";
                if (it->assignedAmbulance) it->assignedAmbulance->isAvailable = true;
                incidents.erase(it);
                break;
            }

            default:
                cout << "[Invalid] Unknown choice!\n";
        }
    }

    return 0;
}
