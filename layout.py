def triangular_to_cartesian(i, j, b):
    x = (i - j/2) * b
    y = j * b * 3**0.5 / 2
    return x, y

def is_inside_hexagon(i, j, n):
    return abs(i) + abs(j) <= n - 1 and abs(i - j) <= n - 1

def circle(center, r, fill='black'):
    return f'<circle cx="{center[0]}" cy="{center[1]}" r="{r}" fill="{fill}" />'

def text(content, insert, text_anchor='middle', font_size='16'):
    return f'<text x="{insert[0]}" y="{insert[1]}" text-anchor="{text_anchor}" font-size="{font_size}">{content}</text>'

def generate_svg(n, b, filename):
    with open(filename, 'w') as f:
        f.write('<?xml version="1.0" encoding="UTF-8" ?>\n')
        f.write('<svg xmlns="http://www.w3.org/2000/svg" version="1.1">\n')

        for i in range(-n+1, n):
            for j in range(-n+1, n):
                if is_inside_hexagon(i, j, n):
                    x, y = triangular_to_cartesian(i, j, b)
                    f.write(circle(center=(x, y), r=b/4))

        f.write(text(f'DF_{n} - {n*(n-1)*3 + 1} poles', insert=(0, -1.2*n*b)))
        f.write('</svg>\n')

baseline_distance = 50
arrangement_count = 5

for n in range(1, arrangement_count + 1):
    generate_svg(n, baseline_distance, f'dancing_forest_{n}.svg')
