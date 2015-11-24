#include "Simulation.h"
#include "printer.h"

Simulation::Simulation(const RigidBody& rigidBody, double tStart, double tEnd, double deltaTime) : _rigidBody{ rigidBody }, _tStart{ tStart }, _tEnd{ tEnd }, _deltaTime{ deltaTime }
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("SDL could not initialize! SDL Error: %s\n", SDL_GetError());
       
    }

}

Simulation::~Simulation()
{
    

    // Clean up
    SDL_Quit();
}

void Simulation::run()
{
    auto *window = SDL_CreateWindow("Spinning top", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 200, 200, SDL_WINDOW_SHOWN);
    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (window == nullptr) {
        // In the case that the window could not be made...
        printf("Could not create window: %s\n", SDL_GetError());
        return;
    }
    auto t = _tStart;

    bool running = true;
    double frameTime = 1.0 / 60.0;

    //run simulation at a fraction of realtime
    _deltaTime = frameTime/10;

    while (running) {
        auto start = SDL_GetTicks();
        //check if key has been pressed -> nudge event
        SDL_Event evnt;
        while(SDL_PollEvent(&evnt))
        {
            if(evnt.type==SDL_KEYDOWN)
            {
                //press spacebar to nudge the spinning top
                if(evnt.key.keysym.sym == SDLK_SPACE)
                {
                    std::cout << "Poking the Spinning top...\n";
                    _nudge = true;
                }
                    
            }
            if (evnt.type == SDL_QUIT)
                running = false;
        }
     
        auto up = _rigidBody.getOrientation()*glm::dvec3{ 0.0,0.0,1.0 };
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        //renders a line that represents the handle of the spinning top as a 2d projection
        SDL_RenderDrawLine(renderer, 100, 190, 100+100*up.y, 190- 100 * up.z);
        SDL_RenderPresent(renderer);
        //printVec(up);
      /*  auto angularVelocity = _rigidBody.getAngularVelocity();
        std::cout << glm::length(angularVelocity)<<std::endl;
        std::cout << std::endl;
        */
        //simulate up to frame time
        
        update(_deltaTime);
        t += _deltaTime;

        auto end = SDL_GetTicks();
        if(end-start<frameTime)
            SDL_Delay(end - start- frameTime);
    }

    SDL_DestroyWindow(window);
}

void Simulation::applyNudge()
{
    _nudge = true;
}

void Simulation::update(double deltaTime)
{
    static double nudgeTimer = 0.0;
    if(_nudge)
    {
        _rigidBody.setTorque({ 50.0,0.0,0.0 });
        nudgeTimer += deltaTime;
        if(nudgeTimer>0.3)
        {
            
            _nudge = false;
            nudgeTimer = 0.0;
        }
        
    }
    _rigidBody.update(deltaTime);
}

RigidBody Simulation::getRigidBody() const
{
    return _rigidBody;
}
