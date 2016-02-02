
#include<iostream>
//SFML header files are at /usr/include/SFML
#include<SFML/Window.hpp>
#include<SFML/Graphics.hpp>
#include<SFML/System.hpp>

int main()
{
	// std::cout << "hello world!"<< std::endl;
	sf::RenderWindow window(sf::VideoMode(800, 600), "SFML window");
	// Limit the framerate to 60 frames per second (this step is optional)
	// window.setFramerateLimit(60);

	// Store the size
	sf::Vector2<int> sizeWindow(window.getSize().x,window.getSize().y);
	// Store the window position
	sf::Vector2<int> posWindow(window.getPosition().x,window.getPosition().y);
	// 
	sf::CircleShape circle;
	circle.setRadius(50);
	circle.setPosition(posWindow.x+10.0,sizeWindow.y/2.0-circle.getRadius());
	if(circle.getPosition().x > window.getPosition().x+window.getSize().x)
	{
		circle.setPosition(window.getPosition().x,circle.getPosition().y);
	}
	if(circle.getPosition().x < window.getPosition().x)
	{
		circle.setPosition(window.getPosition().x+window.getSize().x,circle.getPosition().y);
	}


	std::cout << "================================================" << std::endl;
	std::cout << "PosWindow:\t" << window.getPosition().x << "\t" << window.getPosition().y << std::endl;
	std::cout << "SizeWindow:\t" << sizeWindow.x << "\t" << sizeWindow.y << std::endl;
	std::cout << "Circle:\t" << circle.getPosition().x << "\t" << circle.getPosition().y << std::endl;
	std::cout << "================================================" << std::endl;

	circle.setFillColor(sf::Color::Green);


	sf::Event event;
	int i=0;
	float step = 5;
	while (window.isOpen())
	{
		if(i==0)
		{
			window.clear();
			window.draw(circle);
			window.display();
		}
		i++;
		
		 while(window.pollEvent(event))
		{
			if(event.type==sf::Event::KeyPressed)
			{
				if(event.key.code==sf::Keyboard::Left)
				{
					// left key is pressed: move our character
					std::cout << "Moving left" << std::endl;
					std::cout << "========="<< std::endl;
					circle.setPosition(circle.getPosition().x-step,circle.getPosition().y);
					if(circle.getPosition().x < window.getPosition().x)
					{
						circle.setPosition(window.getPosition().x+window.getSize().x,circle.getPosition().y);
					}

					window.clear();
					window.draw(circle);
					window.display();
				}
				else if(event.key.code==sf::Keyboard::Right)
				{
					std::cout << "Moving right" << std::endl;
					std::cout << "========="<< std::endl;
					circle.setPosition(circle.getPosition().x+step,circle.getPosition().y);
					if(circle.getPosition().x > window.getPosition().x+window.getSize().x)
					{
						circle.setPosition(window.getPosition().x,circle.getPosition().y);
					}

					window.clear();
					window.draw(circle);
					window.display();
				}
				else if(event.key.code==sf::Keyboard::Up)
				{
					std::cout << "Moving forward" << std::endl;
					std::cout << "========="<< std::endl;
					circle.setPosition(circle.getPosition().x,circle.getPosition().y-step);
					if(circle.getPosition().y < window.getPosition().y)
					{
						circle.setPosition(circle.getPosition().x,window.getPosition().y+window.getSize().y);
					}
					window.clear();
					window.draw(circle);
					window.display();
				}
				else if(event.key.code==sf::Keyboard::Down)
				{
					std::cout << "Moving backward" << std::endl;
					std::cout << "========="<< std::endl;
					circle.setPosition(circle.getPosition().x,circle.getPosition().y+step);
					if(circle.getPosition().y > window.getPosition().y+window.getSize().y)
					{
						circle.setPosition(circle.getPosition().x,window.getPosition().y);
					}
					window.clear();
					window.draw(circle);
					window.display();
				}
				else if(event.key.code==sf::Keyboard::Escape)
				{
					window.close();
				}
			}
			if(event.type==sf::Event::KeyReleased)
			{
				std::cout << "Key released !" << std::endl;
			}
			if(event.type==sf::Event::Closed)
			{
				window.close();
			}
	
		}
	}

	return 0;
}














