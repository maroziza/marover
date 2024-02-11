/*
 * ADS1115 test/demo program for the Raspberry PI
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 * Copyright (c) 2013-2022  Bernd Porr <mail@berndporr.me.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <device_kit/Ads1115.hpp>

#include <gpiod.hpp>

#include <iostream>
#include <thread>


int main(int, char * [])
{
	device_kit::Ads1115 ads1115rpi {{"/dev/i2c-1", 0x48}};
	device_kit::Ads1115Settings s;
	s.samplingRate = 64;
	ads1115rpi.configure(s);
	std::cout << "fs = " << s.samplingRate << std::endl;

	// Specify the GPIO chip name and the line number (GPIO 17 in this case)
    const std::string chipname = "/dev/gpiochip0";  // Adjust this based on your GPIO chip
    const unsigned int line_offset = 17;

    // Open the GPIO chip
    auto chip = gpiod::chip(chipname);

    // Get the GPIO line
    auto line = chip.get_line(line_offset);

    // Request events for falling edge
    line.request({
        .consumer = "ads_printer",
        .request_type = gpiod::line_request::EVENT_FALLING_EDGE,
		.flags = {}
    });

    std::cout << "Waiting for GPIO 17 to change state...\n";

    while (true) {
        // Wait for an event
        if (line.event_wait(std::chrono::milliseconds {100}))
        {
            auto const event = line.event_read();
            if (event.event_type == gpiod::line_event::FALLING_EDGE)
			{
                std::cout << "timestamp = " << event.timestamp.count() << ", value = " << ads1115rpi.readValue() << std::endl;
            }
        }
    }

	return 0;
}
