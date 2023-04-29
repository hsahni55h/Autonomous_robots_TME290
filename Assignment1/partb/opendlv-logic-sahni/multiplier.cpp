#include <chrono>
#include <iostream>

#include "cluon-complete.hpp"
#include "messages.hpp"

using namespace std;
using namespace opendlv::proxy;

int32_t main(int32_t, char **) {
	
	// variable initialization
	GroundSpeedReading deliveryMsg;
	
	// receive the request
	cluon::OD4Session od4(132,
	[&od4, &deliveryMsg](cluon::data::Envelope &&envelope) noexcept {
		if (envelope.dataType() == 1091) {
			GroundSpeedRequest receivedMsg = cluon::extractMessage<GroundSpeedRequest>(std::move(envelope));
			
			cout << receivedMsg.groundSpeed() << endl;
			
			// handle the request
			deliveryMsg.groundSpeed((float)2.0 * receivedMsg.groundSpeed());
			od4.send(deliveryMsg);
		}
	});
	
	// send the response
	//od4.send(deliveryMsg);
	
	while (od4.isRunning()) {
		// to avoid high cpu utilization 
		std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
	}
 
	return 0;
}


