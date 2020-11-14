#include "RecordRead.hpp"

RecordRead::RecordRead(std::string MapName,std::string TrackNumber){
    MapName_ = MapName;
    TrackNumber_ = TrackNumber;
    std::string CSV_file = "../recorded_trackfiles/"+MapName+"/vehicle_tracks_"+TrackNumber+".csv";
    Read(CSV_file);
}

void RecordRead::Read(std::string CSV_file){
    for(auto onecar : data_car){
        delete onecar;
    }
    data_car.clear();
    ifstream track_file(CSV_file, ios::in);
    string str;
    getline(track_file, str);

    string track_ID, frame_ID, timestamp_ms, agent_type, x, y, vx, vy, psi_rad, length, width;
    string old_track_ID;

    getline(track_file, track_ID, ',');
    getline(track_file, frame_ID, ',');
    getline(track_file, timestamp_ms, ',');
    getline(track_file, agent_type, ',');
    getline(track_file, x, ',');
    getline(track_file, y, ',');
    getline(track_file, vx, ',');
    getline(track_file, vy, ',');
    getline(track_file, psi_rad, ',');
    getline(track_file, length, ',');
    getline(track_file, width, '\n');

    car* new_car = new class car;
    new_car->id_ = stringToNum<int>(track_ID);
    old_track_ID=track_ID;
    data_car.push_back(new_car);
    state reading_state;
    reading_state.time_ = stringToNum<int>(timestamp_ms);
    reading_state.length_ = stringToNum<double>(length);
    reading_state.width_ = stringToNum<double>(width);
    reading_state.psi_rad_ = stringToNum<double>(psi_rad);
    reading_state.vx_ = stringToNum<double>(vx);
    reading_state.vy_ = stringToNum<double>(vy);
    reading_state.x_ = stringToNum<double>(x);
    reading_state.y_ = stringToNum<double>(y);
    new_car->car_state.push_back(reading_state);

    while (getline(track_file, track_ID, ',')) {
        getline(track_file, frame_ID, ',');
        getline(track_file, timestamp_ms, ',');
        getline(track_file, agent_type, ',');
        getline(track_file, x, ',');
        getline(track_file, y, ',');
        getline(track_file, vx, ',');
        getline(track_file, vy, ',');
        getline(track_file, psi_rad, ',');
        getline(track_file, length, ',');
        getline(track_file, width, '\n');
        if (old_track_ID!=track_ID){
            new_car = new class car;
            new_car->id_ = stringToNum<int>(track_ID);
            old_track_ID=track_ID;
            data_car.push_back(new_car);
        }
        reading_state.time_ = stringToNum<int>(timestamp_ms);
        reading_state.length_ = stringToNum<double>(length);
        reading_state.width_ = stringToNum<double>(width);
        reading_state.psi_rad_ = stringToNum<double>(psi_rad);
        reading_state.vx_ = stringToNum<double>(vx);
        reading_state.vy_ = stringToNum<double>(vy);
        reading_state.x_ = stringToNum<double>(x)-1000;
        reading_state.y_ = stringToNum<double>(y)-1000;
        new_car->car_state.push_back(reading_state);
    }

}