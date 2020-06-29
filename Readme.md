# Fruit Ninja AI

## See this AI in practice

https://www.youtube.com/watch?v=Vw3vU9OdWAs

## Reddit discussion

https://old.reddit.com/r/programming/comments/hdwcfm/i_created_a_perfect_ai_for_fruit_ninja/


## Descripton

The AI only loses when a bomb is overlapped with a fruit on its whole path, as the AI won't find a good opportunity to slice it.

The game as a chrome extension: https://chrome.google.com/webstore/detail/fruit-ninja-game/fdkhnibpmdfmpgaipjiodbpdngccfibp

Simply place the chrome extension on the top right corner of your screen and run this file.

**Some heuristics and timings might differ depending on your machine. The code was tweaked run on a 3.5GHz I5-7600 (no GPU acceleration)**
(too much/little computing time between frames might affect the AI's decisions and timings).
Feel free to update any hardcoded values to better match your machine
## Dependencies
Python 3.8

```
pip install -r requirements.txt
```

## Usage

```
python fruits.py
```

Also save videos in ./tmp :

```
python fruits.py save
```