using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Threading;

namespace Astar
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        static Bitmap bmp;
        static Graphics g;
        const int w = 50, h = 50;
        int szyrzka, wyszka;
        static int scale;
        Thread t, t2;
        static int trackbar = 40;
        static Random rnd = new Random();
        static Stack<pole> stack;

        struct s
        {
            public int x;
            public int y;
        }

        class pole
        {
            public int x;
            public int y;
            public int i;
            public int j;
            public bool top;
            public bool right;
            public bool bottom;
            public bool left;
            public bool goal = false;
            public bool start = false;
            public bool path = false;
            public bool closed = false;
            public bool visited = false;

            public pole(int i, int j)
            {
                this.x = i;
                this.y = j;
                this.i = i * scale;
                this.j = j * scale;
                fScore[j, i] = double.PositiveInfinity;
                gScore[j, i] = double.PositiveInfinity;
                cameFrom[j, i].x = -1;
                cameFrom[j, i].y = -1;
                this.right = (rnd.Next(100) < trackbar);
                this.bottom = (rnd.Next(100) < trackbar);
                if (x >= 1)
                {
                    this.left = grid[y, x - 1].right;
                }
                if(y >= 1)
                {
                    this.top = grid[y - 1, x].bottom;
                }
            }

            public void show()
            {
                if (start)
                {
                    g.FillRectangle(Brushes.LightGreen, i, j, scale, scale);
                }
                else if (goal)
                {
                    g.FillRectangle(Brushes.Red, i, j, scale, scale);
                }
                else if (path)
                {
                    g.FillRectangle(Brushes.MediumSpringGreen, i, j, scale, scale);
                }
                else if(closed)
                {
                    g.FillRectangle(Brushes.LightGray, i, j, scale, scale);
                }

                if(top)
                {
                    g.DrawLine(Pens.Black, i, j, i + scale, j);
                }
                if(right)
                {
                    g.DrawLine(Pens.Black, i + scale, j, i + scale, j + scale);
                }
                if (bottom)
                {
                    g.DrawLine(Pens.Black, i + scale, j + scale, i, j + scale);
                }
                if (left)
                {
                    g.DrawLine(Pens.Black, i, j + scale, i, j);
                }
            }

            public bool checkNeighbors(ref pole next)
            {
                List<pole> neighbors = new List<pole>();
                if (y >= 1 && !grid[y - 1, x].visited) { neighbors.Add(grid[y - 1, x]); }
                if (x < w - 1 && !grid[y, x + 1].visited) { neighbors.Add(grid[y, x + 1]); }
                if (y < h - 1 && !grid[y + 1, x].visited) { neighbors.Add(grid[y + 1, x]); }
                if (x >= 1 && !grid[y, x - 1].visited) { neighbors.Add(grid[y, x - 1]); }

                if (neighbors.Count > 0)
                {
                    next = neighbors[rnd.Next(neighbors.Count)];
                    return true;
                }
                else return false;
            }

            public bool isWallBetween(pole neighbor)
            {
                if (this.x - neighbor.x == 1)
                {
                    return left;
                }
                else if (this.x - neighbor.x == -1)
                {
                    return right;
                }
                else if (this.y - neighbor.y == 1)
                {
                    return top;
                }
                else if (this.y - neighbor.y == -1)
                {
                    return bottom;
                }
                return false;
            }
        }

        double dist(pole a, pole b)
        {
            double d = Math.Abs(a.x - b.x) + Math.Abs(a.y - b.y);
            //double d = Math.Sqrt(Math.Abs(a.x - b.x) * Math.Abs(a.x - b.x) + Math.Abs(a.y - b.y) * Math.Abs(a.y - b.y));
            return d;
        }

        static pole[,] grid = new pole[h, w];
        static double[,] gScore = new double[h, w];
        static double[,] fScore = new double[h, w];
        static s[,] cameFrom = new s[h, w];

        private void Form1_Load(object sender, EventArgs e)
        {
            t = new Thread(Astar);
            t2 = new Thread(generateMaze);
            szyrzka = pictureBox1.Width;
            wyszka = pictureBox1.Height;
            scale = szyrzka / w;
            bmp = new Bitmap(szyrzka, wyszka);
            g = Graphics.FromImage(bmp);
            g.Clear(Color.White);
            
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    grid[j, i] = new pole(i, j);
                }
            }
            grid[0, 0].start = true;
            gScore[0, 0] = 0;
            fScore[0, 0] = dist(grid[0, 0], grid[h - 1, w - 1]);
            namalujPole();
            pictureBox1.Image = bmp;
        }

        void namalujPole()
        {
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    grid[j, i].show();
                }
            }
            pictureBox1.Image = bmp;
        }

        void reconstructPath(pole current)
        {
            int x = current.x;
            int y = current.y;
            grid[y, x].path = true;
            grid[y, x].show();
            while (cameFrom[y, x].x != -1 && cameFrom[y, x].y != -1)
            {
                int pomx = cameFrom[y, x].x;
                y = cameFrom[y, x].y;
                x = pomx;
                grid[y, x].path = true;
                grid[y, x].show();
            }
        }

        void deconstructPath(pole current)
        {
            int x = current.x;
            int y = current.y;
            grid[y, x].path = false;
            grid[y, x].show();
            while (cameFrom[y, x].x != -1 && cameFrom[y, x].y != -1)
            {
                int pomx = cameFrom[y, x].x;
                y = cameFrom[y, x].y;
                x = pomx;
                grid[y, x].path = false;
                grid[y, x].show();
            }
        }

        void Astar()
        {
            List<pole> openSet = new List<pole>();
            List<pole> closedSet = new List<pole>();

            openSet.Add(grid[0, 0]);

            pole goal = grid[h - 1, w - 1];

            while(openSet.Count > 0)
            {
                pole current = openSet[0];
                for(int i = 1; i < openSet.Count; i++)
                {
                    if(fScore[openSet[i].y, openSet[i].x] < fScore[current.y, current.x])
                    {
                        current = openSet[i];
                    }
                }
                
                if (current.Equals(goal))
                {
                    reconstructPath(current);
                    goal.goal = true;
                    goal.show();
                    this.Invoke((MethodInvoker)delegate
                    {
                        pictureBox1.Image = bmp;
                        pictureBox1.Update();
                        pictureBox1.Show();
                        button2.Enabled = true;
                        button3.Enabled = false;
                    });
                    return;
                }

                openSet.Remove(current);
                closedSet.Add(current);
                grid[current.y, current.x].closed = true;
                reconstructPath(current);
                this.Invoke((MethodInvoker)delegate
                {
                    pictureBox1.Image = bmp;
                    pictureBox1.Update();
                    pictureBox1.Show();
                });
                deconstructPath(current);

                for (int i = 0; i < 9; i++)
                {
                    int x = current.x + (i % 3) - 1;
                    int y = current.y + (i / 3) - 1;

                    if ((i % 2) != 0 && i != 4 && x >= 0 && x < w && y >= 0 && y < h && /*!grid[y, x].wall*/ !current.isWallBetween(grid[y, x]))
                    {
                        pole neighbor = grid[y, x];
                        if (closedSet.Contains(neighbor))
                        {
                            continue;
                        }
                        double tentativeGScore = gScore[current.y, current.x] + dist(current, neighbor);

                        if (!openSet.Contains(neighbor))
                        {
                            openSet.Add(neighbor);
                        }
                        else if (tentativeGScore >= gScore[y, x])
                        {
                            continue;
                        }
                        cameFrom[y, x].x = current.x;
                        cameFrom[y, x].y = current.y;
                        gScore[y, x] = tentativeGScore;
                        fScore[y, x] = gScore[y, x] + dist(neighbor, goal);
                    }
                }
            }
            this.Invoke((MethodInvoker)delegate
            {
                button2.Enabled = true;
                button3.Enabled = false;
            });
        }

        void removeWalls(pole current, pole next)
        {
            if (current.x - next.x == 1)
            {
                grid[current.y, current.x].left = grid[next.y, next.x].right = false;
                g.DrawLine(Pens.White, current.i, current.j + scale, current.i, current.j);
            }
            else if (current.x - next.x == -1)
            {
                grid[current.y, current.x].right = grid[next.y, next.x].left = false;
                g.DrawLine(Pens.White, current.i + scale, current.j, current.i + scale, current.j + scale);
            }
            else if (current.y - next.y == 1)
            {
                grid[current.y, current.x].top = grid[next.y, next.x].bottom = false;
                g.DrawLine(Pens.White, current.i, current.j, current.i + scale, current.j);
            }
            else if (current.y - next.y == -1)
            {
                grid[current.y, current.x].bottom = grid[next.y, next.x].top = false;
                g.DrawLine(Pens.White, current.i + scale, current.j + scale, current.i, current.j + scale);
            }
        }

        void generateMaze()
        {
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    grid[j, i].top = grid[j, i].bottom = grid[j, i].left = grid[j, i].right = true;
                }
            }
            pole current = grid[0, 0];
            int unvisited = w * h;
            stack = new Stack<pole>();
            grid[current.y, current.x].visited = true;
            unvisited--;
            while (unvisited > 0)
            {
                pole next = current;
                if (current.checkNeighbors(ref next))
                {
                    stack.Push(current);
                    removeWalls(current, next);
                    
                    current = next;
                    grid[current.y, current.x].visited = true;
                    grid[current.y, current.x].show();

                    unvisited--;
                }
                else if(stack.Count > 0)
                {
                    current = stack.Pop();
                }
            }
            namalujPole();
            this.Invoke((MethodInvoker)delegate
            {
                pictureBox1.Image = bmp;
                pictureBox1.Update();
                pictureBox1.Show();
                button1.Enabled = true;
                button2.Enabled = true;
                buttonGenerateMaze.Enabled = true;
            });
        }

        //reset
        private void button2_Click(object sender, EventArgs e)
        {
            t = new Thread(Astar);
            t2 = new Thread(generateMaze);
            button1.Enabled = true;
            buttonGenerateMaze.Enabled = true;
            g.Clear(Color.White);
            Random rnd = new Random();
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    grid[j, i] = new pole(i, j);
                }
            }
            grid[0, 0].start = true;
            gScore[0, 0] = 0;
            fScore[0, 0] = dist(grid[0, 0], grid[h - 1, w - 1]);
            namalujPole();
            pictureBox1.Image = bmp;
        }
        
        //stop
        private void button3_Click(object sender, EventArgs e)
        {
            if (t.IsAlive)
            {
                button1.Enabled = true;
                button2.Enabled = true;
                button3.Enabled = false;
                t.Suspend();
            }
        }
        
        //start
        private void button1_Click(object sender, EventArgs e)
        {
            button1.Enabled = false;
            button2.Enabled = false;
            button3.Enabled = true;
            buttonGenerateMaze.Enabled = false;
            if (t.IsAlive)
            {
                t.Resume();
            }
            else
            {
                t.Start();
            }
        }

        private void buttonGenerateMaze_Click(object sender, EventArgs e)
        {
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    grid[j, i] = new pole(i, j);
                }
            }
            buttonGenerateMaze.Enabled = false;
            button1.Enabled = false;
            button2.Enabled = false;
            button3.Enabled = false;
            generateMaze();
            //t2.Start();
        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            label1.Text = "Wall probability: " + trackBar1.Value + "%";
            trackbar = trackBar1.Value;
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            t.Abort();
            t2.Abort();
            Thread.Sleep(50);
        }
    }
}
